//! The Webots controller: a plain bus client that simulates one robot.
//!
//! Binds one Webots-owned controller process to a robot's component
//! capabilities and publishes or subscribes exactly the `component::*`
//! contracts those capabilities need. It is not a Phoxal participant: there is
//! no runner, no role attribute and no setup context. It reads the bundle's
//! `manifest.json`, learns its execution from the router, opens an external bus
//! session, mints one opaque timeline, and runs the external Webots step loop.
//!
//! Every capability kind a component may declare is simulated except one:
//! Webots has no button, switch, or toggle node, so nothing in a simulated
//! world can engage or release an `emergency_stop`. That capability is
//! deliberately left unpublished rather than driven from a static config,
//! which would assert a state no one in the world can change.

use std::path::Path;

use anyhow::{Context, Result, bail};
use phoxal_bundle::RuntimeBundle;
use phoxal_bus::{
    BusConfig, BusOwner, ParticipantId, ParticipantReadyToken, SourceLabel, TimelineAuthority,
    TimelineId, WorldClockPublisher, WorldStepToken,
};
use phoxal_protocol::runtime::endpoint::simulation::ClockEndpoint;
use phoxal_protocol::runtime::simulation::Clock;

use crate::backend::{SharedBackend, WebotsHandle};
use crate::catalog::CapabilityCatalog;
use crate::channel::{CapabilityChannel, StepOutput};
use crate::runtime::ControllerRuntime;

/// The diagnostic label this session's samples carry. It never affects
/// routing, authority, or Ready admission - it only says which external client
/// produced a sample when someone reads the metadata.
const SOURCE_LABEL: &str = "webots-controller";

/// The one contract this controller publishes as itself rather than on behalf
/// of a device. Every capability's own handle lives with the device serving it.
#[derive(Clone)]
pub(crate) struct WorldClock {
    clock: WorldClockPublisher<ClockEndpoint>,
}

impl WorldClock {
    /// Publish everything one completed world advance produced, then the clock
    /// that closes it. The order is the contract: a reader that has seen the
    /// clock for a step has already seen that step's outputs.
    pub(crate) fn commit_step(
        &self,
        world_step: &WorldStepToken,
        step: u64,
        output: StepOutput,
    ) -> Result<()> {
        output.publish(world_step)?;
        self.clock.publish(world_step, Clock { step })?;
        Ok(())
    }
}

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
        channels.push(CapabilityChannel::bind(&bus, handle.webots(), spec).await?);
    }

    // Presence is declared only once every channel is bound: a driver that
    // reads as present must already be able to serve its contracts.
    //
    // One token per simulated component instance, and none for the controller
    // itself. The supervisor watches presence per participant id and derives
    // the expected set from the manifest, where a driver's participant id IS
    // its component instance id; this process stands in for all of them, and it
    // is not itself an expected runtime.
    let mut presence: Vec<ParticipantReadyToken> = Vec::new();
    for component in robot.components() {
        let participant = ParticipantId::new(component.id().as_str())?;
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
        components = presence.len(),
        "webots controller ready"
    );

    let backend = SharedBackend::new(handle.into_backend(channels));
    let clock = WorldClock {
        clock: WorldClockPublisher::mint(
            bus.clone(),
            &phoxal_protocol::runtime::topic::owner()
                .simulation()
                .clock(),
        )?,
    };
    let runtime = ControllerRuntime::new(
        TimelineAuthority::mint(TimelineId::mint())?,
        backend.clone(),
    );

    let outcome = tokio::select! {
        result = runtime.run(clock) => result,
        () = crate::shutdown_signal() => {
            tracing::info!(
                target: crate::LOG_TARGET,
                "termination signal received; stopping the Webots controller"
            );
            Ok(())
        }
    };

    // Quiet the world first, then let go of the presence this process was
    // standing in for, then close the session. Dropping presence before the
    // motors are parked would let a reader believe the drivers are gone while
    // the wheels are still turning.
    let parked = backend.park().await;
    drop(presence);
    owner.close().await;

    outcome.and(parked)
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
    use crate::channel::PendingPublish;
    use phoxal_bus::{ExecutionId, RobotInstant, SamplePublisher, SampleReceiver, StreamReceiver};
    use phoxal_protocol::robot as api;
    use std::time::Duration;

    /// One process may mint exactly one timeline authority, so the commit
    /// path's behaviour is covered by this single test rather than one per
    /// assertion. It runs on a real in-process bus session opened exactly the
    /// way the controller opens its own - `for_external`, no participant
    /// identity - because the ordering under test is the transport's, not this
    /// module's arithmetic.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_step_publishes_its_outputs_before_its_clock() {
        let (owner, bus) = BusOwner::open(BusConfig::for_external(
            ExecutionId::mint(),
            Some(SourceLabel::new(SOURCE_LABEL).expect("a bounded label")),
            Vec::new(),
        ))
        .await
        .expect("bus should open");
        let clock_subscriber = StreamReceiver::<ClockEndpoint>::new(
            &bus,
            &phoxal_protocol::runtime::topic::client()
                .simulation()
                .clock(),
        )
        .await
        .expect("clock subscriber should attach");
        let encoder_subscriber =
            SampleReceiver::<api::endpoint::component::encoder::SampleEndpoint>::new(
                &bus,
                &api::topic::client()
                    .component("left_drive")
                    .expect("valid component segment")
                    .encoder("encoder")
                    .expect("valid capability segment")
                    .sample(),
            )
            .await
            .expect("encoder subscriber should attach");
        let encoder_publisher = SamplePublisher::new(
            bus.clone(),
            &api::topic::owner()
                .component("left_drive")
                .expect("valid component segment")
                .encoder("encoder")
                .expect("valid capability segment")
                .sample(),
        )
        .expect("encoder publisher should attach");
        let world_clock = WorldClock {
            clock: WorldClockPublisher::mint(
                bus.clone(),
                &phoxal_protocol::runtime::topic::owner()
                    .simulation()
                    .clock(),
            )
            .expect("clock publisher should attach"),
        };

        let timeline = TimelineId::from_raw(77).expect("test timeline must be nonzero");
        let authority = TimelineAuthority::mint(timeline).expect("authority should mint");
        let world_step = authority.completed_step(20_000_000);
        let output = StepOutput::new(vec![PendingPublish::Encoder(
            encoder_publisher,
            api::component::encoder::Sample::try_new(1.0, 0.5).unwrap(),
        )]);
        world_clock
            .commit_step(&world_step, 2, output)
            .expect("commit should publish");

        let encoder = tokio::time::timeout(Duration::from_secs(2), encoder_subscriber.recv())
            .await
            .expect("encoder output should arrive")
            .expect("encoder output should decode");
        let clock = tokio::time::timeout(Duration::from_secs(2), clock_subscriber.recv())
            .await
            .expect("clock should arrive")
            .expect("clock should decode");

        // Every output of one completed world step shares that step's exact
        // instant, and it rides in the envelope rather than in any body.
        let expected = RobotInstant::new(timeline, 20_000_000);
        assert_eq!(encoder.metadata.produced_exactly_at(), Some(expected));
        assert_eq!(clock.metadata.produced_exactly_at(), Some(expected));
        assert_eq!(clock.body.step, 2);
        assert!(
            encoder.metadata.sequence < clock.metadata.sequence,
            "all completed-world outputs must enqueue before the matching clock"
        );
        owner.close().await;
    }
}
