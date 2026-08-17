//! The Webots controller: a plain bus client that simulates one robot.
//!
//! Binds one Webots-owned controller process to a robot's component
//! capabilities and publishes or subscribes exactly the `component::*`
//! contracts those capabilities need. It is not a Phoxal participant: there is
//! no runner, no role attribute and no setup context. It reads the bundle's
//! `manifest.json`, learns its execution from the router, opens an external bus
//! session, mints one opaque timeline, and runs the external Webots step loop.
//!
//! Three capability kinds are not simulated. Webots has no button, switch, or
//! toggle node, so nothing in a simulated world can engage or release an
//! `emergency_stop`: it is deliberately left unpublished rather than driven
//! from a static config, which would assert a state no one in the world can
//! change. `led` and `speaker` are refused outright at startup, because no
//! participant owns those effects in the current graph.

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use anyhow::{Context, Result, anyhow, bail};
use phoxal_bundle::RuntimeBundle;
use phoxal_bus::{
    BusConfig, BusOwner, ParticipantId, ParticipantReadyToken, SourceLabel, TimelineAuthority,
    TimelineId, WorldClockPublisher,
};
use phoxal_model::Robot;
use phoxal_model::identity::ComponentInstanceId;

use crate::backend::WebotsHandle;
use crate::catalog::CapabilityCatalog;
use crate::channel::CapabilityChannel;
use crate::runtime::ControllerRuntime;

/// The diagnostic label this session's samples carry. It never affects
/// routing, authority, or Ready admission - it only says which external client
/// produced a sample when someone reads the metadata.
const SOURCE_LABEL: &str = "webots-controller";

/// The name the step loop's thread carries in a debugger, a crash report and
/// `ps -T`. There is exactly one of it for the life of the process.
const STEP_LOOP_THREAD: &str = "webots-step-loop";

/// Run the controller until the world stops or the host asks it to.
pub(crate) async fn run(bundle_root: &Path, connect: &str) -> Result<()> {
    let bundle = RuntimeBundle::open(bundle_root)
        .with_context(|| format!("failed to open the bundle at {}", bundle_root.display()))?;
    let robot = bundle.robot();

    // Open Webots before resolving the catalog: the world's actual
    // basicTimeStep determines device-period quantization and therefore each
    // capability's effective publish cadence.
    let handle = WebotsHandle::open()?;
    let catalog = CapabilityCatalog::from_robot(robot, handle.basic_time_step_ms())?;

    let execution = sole_execution(connect).await?;
    let label = SourceLabel::new(SOURCE_LABEL)?;
    let (owner, bus) = BusOwner::open(BusConfig::for_external(
        execution,
        Some(label),
        vec![connect.to_string()],
    ))
    .await
    .with_context(|| format!("failed to open the controller bus session on {connect}"))?;

    // One pass over the catalog binds each capability's device and its bus
    // handle together, so the two are never matched up by position later.
    let mut channels = Vec::with_capacity(catalog.specs().len());
    for spec in catalog.specs() {
        channels.push(
            CapabilityChannel::bind(&bus, handle.webots(), spec)
                .await
                .with_context(|| {
                    format!(
                        "failed to bind the {} capability {} the bundle manifest declares",
                        spec.kind().as_str(),
                        spec.reference()
                    )
                })?,
        );
    }

    // Presence is declared only once every channel is bound: a driver that
    // reads as present must already be able to serve its contracts.
    let mut presence: Vec<ParticipantReadyToken> = Vec::new();
    for participant in presented_participants(robot) {
        let participant = ParticipantId::new(participant.as_str())?;
        presence.push(
            owner
                .declare_participant_ready_as(&participant)
                .await
                .with_context(|| format!("failed to declare presence for {participant}"))?,
        );
    }

    tracing::info!(
        target: crate::LOG_TARGET,
        %execution,
        robot = %robot.id(),
        capabilities = ?catalog.kind_counts(),
        presented = presence.len(),
        "webots controller ready"
    );

    let clock = WorldClockPublisher::mint(
        bus.clone(),
        &phoxal_protocol::runtime::topic::owner()
            .simulation()
            .clock(),
    )?;
    let stop = Arc::new(AtomicBool::new(false));
    let runtime = ControllerRuntime::new(
        TimelineAuthority::mint(TimelineId::mint())?,
        clock,
        handle.into_backend(channels),
        Arc::clone(&stop),
    );

    // The step loop gets one dedicated OS thread for the life of the process.
    // Every Webots call blocks and every one of them must come from the thread
    // that opened the devices, so this thread owns the world outright: it reads
    // each capability and publishes it in place, with nothing carried back
    // across a task boundary and no publisher cloned per sample. Publishing is
    // a synchronous enqueue onto the bus session's outbound lane, so it needs
    // no runtime handle here; the transport drains that lane on Tokio's own
    // threads, which keep running while this one is inside Webots.
    let (done, thread_ended) = tokio::sync::oneshot::channel::<()>();
    let step_loop = std::thread::Builder::new()
        .name(STEP_LOOP_THREAD.to_owned())
        .spawn(move || {
            // Nothing is ever sent on this channel: dropping the sender as this
            // thread returns is what wakes the async side, and it does so
            // whether the loop ended, failed, or panicked.
            let _done = done;
            runtime.run()
        })
        .context("failed to start the Webots step loop thread")?;

    tokio::select! {
        _ = thread_ended => {}
        () = crate::shutdown_signal() => {
            tracing::info!(
                target: crate::LOG_TARGET,
                "termination signal received; stopping the Webots controller"
            );
            stop.store(true, Ordering::Release);
        }
    }

    // Joining is what quiets the world: the loop parks every device before it
    // returns, and it reaches that point within one world step of the flag.
    // Only then does this process let go of the presence it was standing in for
    // and close the session - dropping presence while the wheels were still
    // turning would let a reader believe the drivers are already gone.
    let outcome = step_loop
        .join()
        .map_err(|_| anyhow!("the Webots step loop thread panicked"))?;
    drop(presence);
    owner.close().await;

    outcome
}

/// The component instances this controller stands in for, in the robot's
/// canonical instance order.
///
/// Exactly the instances that declare a `driver` block, which is the same
/// derivation every launcher applies to decide which driver processes a real
/// robot starts. A component without a driver block has no process on a real
/// robot either, so nothing expects it to be present and nothing here declares
/// it: presenting one would put a participant id on the bus that the supervisor
/// never asked about, and a simulated robot would read as *more* complete than
/// the same robot on hardware.
///
/// The controller itself is never in this set. The supervisor watches presence
/// per participant id and a driver's participant id IS its component instance
/// id; this one process stands in for all of them and is not itself an expected
/// runtime.
fn presented_participants(robot: &Robot) -> impl Iterator<Item = &ComponentInstanceId> {
    robot
        .components()
        .filter(|component| component.instance().driver().is_some())
        .map(|component| component.id())
}

/// The execution this controller joins, learned from the router it connects to.
///
/// The execution id is not in argv and never will be: a router's Zenoh session
/// id IS the execution, so asking the transport is the only answer that cannot
/// disagree with the run actually in progress.
async fn sole_execution(connect: &str) -> Result<phoxal_bus::ExecutionId> {
    let executions = BusOwner::probe_routers(connect)
        .await
        .with_context(|| format!("failed to probe the router at {connect}"))?;
    match executions.as_slice() {
        [execution] => Ok(*execution),
        [] => bail!(
            "no Phoxal router answered at {connect}; start the supervisor before the simulation"
        ),
        many => bail!(
            "{connect} reports {} executions ({}); connect to exactly one supervisor's router",
            many.len(),
            many.iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", ")
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phoxal_model::RobotBuilder;

    /// A robot whose `wheel` instance is driven by a component driver and whose
    /// `bumper` instance is not.
    ///
    /// The builder states everything except the driver block, which no
    /// in-memory constructor exposes, so the block is written into the same
    /// persisted document a bundle carries and read back through the model's own
    /// validating deserializer.
    fn robot_with_one_driven_component() -> Robot {
        let robot = RobotBuilder::new("presence-rover")
            .component_type("wheel_drive", |wheel| wheel.motor("motor", "axle"))
            .component_type("bumper_pad", |bumper| bumper.range("range", "bumper_link"))
            .component("wheel", "wheel_drive")
            .component("bumper", "bumper_pad")
            .build()
            .expect("the presence model must build");

        let mut document = serde_json::to_value(&robot).expect("a robot must serialize");
        document["components"]["wheel"]["driver"] =
            serde_json::json!({ "connection": { "type": "can", "bus": 0, "node_id": 1 } });
        serde_json::from_value(document).expect("the patched manifest must still validate")
    }

    /// Ruling: the presented set is exactly the manifest's driver set, which is
    /// what the supervisor expects. A component with no driver block runs no
    /// process on a real robot, so a simulated one declares no presence for it
    /// either.
    #[test]
    fn only_components_with_a_driver_block_are_presented() {
        let robot = robot_with_one_driven_component();
        assert_eq!(
            robot
                .components()
                .map(|component| component.id().as_str())
                .collect::<Vec<_>>(),
            ["bumper", "wheel"],
            "both components stay mounted, bound and simulated"
        );
        assert_eq!(
            presented_participants(&robot)
                .map(ComponentInstanceId::as_str)
                .collect::<Vec<_>>(),
            ["wheel"]
        );
    }
}
