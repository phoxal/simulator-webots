//! The Webots controller: an external simulator host that simulates one robot.
//!
//! Binds one Webots-owned controller process to a robot's component
//! capabilities and publishes or subscribes exactly the `component::*`
//! contracts those capabilities need. It is not a Phoxal participant: there is
//! no runner, no role attribute and no setup context. It reads the bundle's
//! `manifest.json`, attaches to the one execution reachable at `--connect`
//! through [`SimulatorSession`], and runs the external Webots step loop against
//! the world time that session hands it.
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

use anyhow::{Context, Result, anyhow};
use phoxal::bundle::RuntimeBundle;
use phoxal::identity::ParticipantId;
use phoxal::model::Robot;
use phoxal::model::identity::ComponentInstanceId;
use phoxal::simulator::{SimulatorConnectOptions, SimulatorSession};

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

    // The execution is learned, not stated: `connect` must identify exactly one
    // of them, and the session refuses to open when it identifies none or
    // several.
    let mut session =
        SimulatorSession::connect(SimulatorConnectOptions::new(connect, SOURCE_LABEL))
            .await
            .with_context(|| format!("failed to attach the Webots controller at {connect}"))?;
    let execution = session.execution();

    // One pass over the catalog binds each capability's device and its bus
    // handle together, so the two are never matched up by position later.
    let mut channels = Vec::with_capacity(catalog.specs().len());
    for spec in catalog.specs() {
        channels.push(
            CapabilityChannel::bind(&session, handle.webots(), spec)
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
    // reads as present must already be able to serve its contracts. The session
    // holds each delegated lease and drops them all when it closes.
    let presented = presented_participants(robot)
        .map(|instance| ParticipantId::new(instance.as_str()))
        .collect::<Result<Vec<_>, _>>()?;
    for participant in &presented {
        session
            .present(participant)
            .await
            .with_context(|| format!("failed to declare presence for {participant}"))?;
    }

    tracing::info!(
        target: crate::LOG_TARGET,
        %execution,
        robot = %robot.id(),
        capabilities = ?catalog.kind_counts(),
        presented = presented.len(),
        "webots controller ready"
    );

    // The world's time is taken once and moves to the step-loop thread below:
    // the timeline this process owns and the clock hand that closes each step
    // belong to the thread that advances the world.
    let world_time = session.take_world_time()?;
    let stop = Arc::new(AtomicBool::new(false));
    let runtime =
        ControllerRuntime::new(world_time, handle.into_backend(channels), Arc::clone(&stop));

    // The step loop gets one dedicated OS thread for the life of the process.
    // Every Webots call blocks and every one of them must come from the thread
    // that opened the devices, so this thread owns the world outright: it reads
    // each capability and publishes it in place, with nothing carried back
    // across a task boundary and no publisher cloned per sample. Publishing is
    // a synchronous enqueue onto the session's outbound lane, so it needs
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
    // Only then does this process close the session, which drops the presence
    // it was standing in for and then the transport - closing while the wheels
    // were still turning would let a reader believe the drivers are already
    // gone.
    let outcome = step_loop
        .join()
        .map_err(|_| anyhow!("the Webots step loop thread panicked"))?;
    session.close().await;

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

#[cfg(test)]
mod tests {
    use super::*;
    use phoxal::model::RobotBuilder;

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
