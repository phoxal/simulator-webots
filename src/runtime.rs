//! The external Webots step loop.
//!
//! Webots owns the cadence, so this controller runs no framework step loop of
//! its own. Each iteration applies the actuator inputs, advances the world one
//! step, publishes everything that step produced, and closes it with the world
//! clock.
//!
//! The loop is synchronous and runs on one dedicated OS thread for the life of
//! the process: every Webots call blocks, and owning the world outright is what
//! lets a step publish each sample on its own handle in place, rather than
//! carrying the step's bodies back to an async task that then has to say which
//! contract each of them belongs to.

use std::ops::ControlFlow;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use anyhow::Result;
use phoxal_bus::{TimelineAuthority, TimelineId, WorldClockPublisher, WorldStepToken};
use phoxal_protocol::runtime::endpoint::simulation::ClockEndpoint;
use phoxal_protocol::runtime::simulation::Clock;

use crate::backend::Advance;
use crate::capabilities::SensorStep;

/// The world one turn of the step loop drives.
///
/// The loop is written against this rather than against
/// [`WebotsBackend`](crate::backend::WebotsBackend) directly for one reason: a
/// `WebotsBackend` exists only inside a live Webots controller process, while
/// the ordering this loop guarantees - every output of a step published before
/// the clock that closes it - is exactly what has to stay covered without one.
/// There is a single production implementation.
pub(crate) trait StepWorld {
    /// Apply the graph's inputs and advance the world one step.
    fn advance(&mut self) -> Result<Advance>;

    /// Publish everything the completed step produced, stamped with its token.
    fn publish_due(&mut self, step: SensorStep, world_step: &WorldStepToken) -> Result<()>;

    /// Leave the world quiet.
    fn park(&mut self) -> Result<()>;
}

pub(crate) struct ControllerRuntime<W: StepWorld> {
    /// This controller's exclusive ownership of the world's timeline. It is
    /// the only way anything in this process can express a robot instant.
    authority: TimelineAuthority,
    /// The one contract this controller publishes as itself rather than on
    /// behalf of a device. Every capability's own handle lives with the device
    /// serving it.
    clock: WorldClockPublisher<ClockEndpoint>,
    step_index: u64,
    world: W,
    /// Set by the host when a signal asks this controller to stop. The loop
    /// reads it between steps, so a stop takes effect at a step boundary and
    /// never inside one.
    stop: Arc<AtomicBool>,
}

impl<W: StepWorld> ControllerRuntime<W> {
    pub(crate) const fn new(
        authority: TimelineAuthority,
        clock: WorldClockPublisher<ClockEndpoint>,
        world: W,
        stop: Arc<AtomicBool>,
    ) -> Self {
        Self {
            authority,
            clock,
            step_index: 0,
            world,
            stop,
        }
    }

    /// Step the world until it stops.
    ///
    /// However it stops - the host asking this controller to stop, Webots
    /// asking it to shut down, or a device failing - the world is left quiet
    /// before this returns, so a stopped loop never leaves the motors running.
    /// Only the last is an error: a reverted or quit world is how a simulation
    /// ends.
    pub(crate) fn run(mut self) -> Result<()> {
        loop {
            if self.stop.load(Ordering::Acquire) {
                tracing::info!(
                    target: crate::LOG_TARGET,
                    steps = self.step_index,
                    "the host asked the Webots step loop to stop"
                );
                return self.world.park();
            }
            match self.step_once() {
                Ok(ControlFlow::Continue(())) => {}
                Ok(ControlFlow::Break(())) => {
                    tracing::info!(
                        target: crate::LOG_TARGET,
                        steps = self.step_index,
                        "Webots asked the controller to shut down"
                    );
                    return self.world.park();
                }
                Err(error) => {
                    if let Err(park_error) = self.world.park() {
                        tracing::warn!(
                            target: crate::LOG_TARGET,
                            error = %park_error,
                            "failed to quiet the world after the Webots step loop stopped"
                        );
                    }
                    return Err(error);
                }
            }
        }
    }

    /// Advance the world once and commit what it produced.
    ///
    /// The order is the contract: the world advances, the instant it reached
    /// mints this step's token, every capability publishes what it read at that
    /// instant, and only then does the clock close the step. A reader that has
    /// seen a step's clock has already seen that step's outputs.
    fn step_once(&mut self) -> Result<ControlFlow<()>> {
        let Advance::Stepped { time_ns, rewound } = self.world.advance()? else {
            return Ok(ControlFlow::Break(()));
        };
        if rewound {
            self.authority.replace_timeline(TimelineId::mint());
            self.step_index = 0;
            tracing::info!(
                target: crate::LOG_TARGET,
                "Webots time rewound; replaced the world timeline and re-anchored schedules"
            );
        }
        let next_step = self
            .step_index
            .checked_add(1)
            .ok_or_else(|| anyhow::anyhow!("Webots controller step counter exhausted"))?;

        // One completed world advance mints one token, and every output of
        // that advance is stamped with it. There is no other way for this
        // process to express a robot instant.
        let world_step = self.authority.completed_step(time_ns);
        self.world
            .publish_due(SensorStep { time_ns }, &world_step)?;
        self.clock.publish(&world_step, Clock { step: next_step })?;
        self.step_index = next_step;
        tracing::trace!(
            target: crate::LOG_TARGET,
            timeline = %self.authority.timeline(),
            step = self.step_index,
            ticks = time_ns,
            "external Webots step committed"
        );
        Ok(ControlFlow::Continue(()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phoxal_bus::{
        BusConfig, BusHandle, BusOwner, CaptureStamp, ExecutionId, RobotInstant, SamplePublisher,
        SampleReceiver, SourceLabel, StepStamp, StreamReceiver,
    };
    use phoxal_protocol::robot as api;
    use std::collections::VecDeque;
    use std::time::Duration;

    /// A world scripted advance by advance, standing in for the Webots one.
    ///
    /// It publishes one encoder sample per step on a real handle, so the
    /// ordering below is asserted against the transport the controller actually
    /// uses rather than against a recording of intent.
    struct ScriptedWorld {
        advances: VecDeque<Result<Advance>>,
        encoder: SamplePublisher<api::endpoint::component::encoder::SampleEndpoint>,
        /// Set once a step has published, standing in for a signal arriving
        /// while the loop is mid-step.
        stop_after_publish: Option<Arc<AtomicBool>>,
        /// Observed after the runtime that owns this world has been consumed.
        parked: Arc<AtomicBool>,
    }

    impl StepWorld for ScriptedWorld {
        fn advance(&mut self) -> Result<Advance> {
            self.advances.pop_front().unwrap_or(Ok(Advance::Stopped))
        }

        fn publish_due(&mut self, step: SensorStep, world_step: &WorldStepToken) -> Result<()> {
            self.encoder.publish(
                CaptureStamp::exact(world_step.instant()),
                api::component::encoder::Sample::try_new(step.time_ns as f64, 0.5)?,
            )?;
            if let Some(stop) = &self.stop_after_publish {
                stop.store(true, Ordering::Release);
            }
            Ok(())
        }

        fn park(&mut self) -> Result<()> {
            self.parked.store(true, Ordering::Release);
            Ok(())
        }
    }

    /// What one scripted run ended as, read after the runtime - and with it
    /// this process's one timeline authority - has been dropped.
    struct Scenario {
        outcome: Result<()>,
        parked: bool,
        timeline: TimelineId,
    }

    /// Run one scripted world to completion on a real bus session.
    fn run_scenario(
        bus: &BusHandle,
        advances: Vec<Result<Advance>>,
        stop: &Arc<AtomicBool>,
        stop_after_publish: Option<Arc<AtomicBool>>,
    ) -> Scenario {
        let parked = Arc::new(AtomicBool::new(false));
        let world = ScriptedWorld {
            advances: advances.into_iter().collect(),
            encoder: SamplePublisher::new(
                bus.clone(),
                &api::topic::owner()
                    .component("left_drive")
                    .expect("valid component segment")
                    .encoder("encoder")
                    .expect("valid capability segment")
                    .sample(),
            )
            .expect("encoder publisher should attach"),
            stop_after_publish,
            parked: Arc::clone(&parked),
        };
        let clock = WorldClockPublisher::mint(
            bus.clone(),
            &phoxal_protocol::runtime::topic::owner()
                .simulation()
                .clock(),
        )
        .expect("clock publisher should attach");
        let authority =
            TimelineAuthority::mint(TimelineId::mint()).expect("the authority slot must be free");
        let timeline = authority.timeline();
        let outcome = ControllerRuntime::new(authority, clock, world, Arc::clone(stop)).run();
        Scenario {
            outcome,
            parked: parked.load(Ordering::Acquire),
            timeline,
        }
    }

    /// One process may mint exactly one timeline authority, so every way the
    /// loop can stop is covered by this single test, each scenario dropping its
    /// runtime - and with it the authority - before the next one mints. The bus
    /// session is opened exactly the way the controller opens its own
    /// (`for_external`, no participant identity), because the ordering under
    /// test is the transport's rather than this module's arithmetic.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_step_publishes_its_outputs_before_its_clock_and_every_stop_parks_the_world() {
        let (owner, bus) = BusOwner::open(BusConfig::for_external(
            ExecutionId::mint(),
            Some(SourceLabel::new("webots-controller").expect("a bounded label")),
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

        // Webots quits the world after one step: the step is committed, and the
        // world is quiet before the loop returns.
        let stopped = run_scenario(
            &bus,
            vec![Ok(Advance::Stepped {
                time_ns: 20_000_000,
                rewound: false,
            })],
            &Arc::new(AtomicBool::new(false)),
            None,
        );
        assert!(stopped.outcome.is_ok(), "a quit world is not a failure");
        assert!(stopped.parked, "a stopped world must be left quiet");

        let encoder = tokio::time::timeout(Duration::from_secs(2), encoder_subscriber.recv())
            .await
            .expect("encoder output should arrive")
            .expect("encoder output should decode");
        let tick = tokio::time::timeout(Duration::from_secs(2), clock_subscriber.recv())
            .await
            .expect("clock should arrive")
            .expect("clock should decode");

        // Every output of one completed world step shares that step's exact
        // instant, and it rides in the envelope rather than in any body.
        let expected = RobotInstant::new(stopped.timeline, 20_000_000);
        assert_eq!(encoder.metadata.produced_exactly_at(), Some(expected));
        assert_eq!(tick.metadata.produced_exactly_at(), Some(expected));
        assert_eq!(tick.body.step, 1);
        assert!(
            encoder.metadata.sequence < tick.metadata.sequence,
            "all completed-world outputs must enqueue before the matching clock"
        );

        // A host signal stops the loop at the next step boundary, before the
        // second scripted advance, and that world is left quiet too.
        let stop = Arc::new(AtomicBool::new(false));
        let signalled = run_scenario(
            &bus,
            vec![
                Ok(Advance::Stepped {
                    time_ns: 20_000_000,
                    rewound: false,
                }),
                Err(anyhow::anyhow!("the loop must never reach this advance")),
            ],
            &stop,
            Some(Arc::clone(&stop)),
        );
        assert!(
            signalled.outcome.is_ok(),
            "a requested stop is not a failure"
        );
        assert!(signalled.parked, "a signalled world must be left quiet");

        // A failing world is reported, and parked first: a motor left running
        // is worse than an error nobody sees.
        let failed = run_scenario(
            &bus,
            vec![Err(anyhow::anyhow!("device rejected a command"))],
            &Arc::new(AtomicBool::new(false)),
            None,
        );
        assert_eq!(
            failed
                .outcome
                .expect_err("a device failure must reach the caller")
                .to_string(),
            "device rejected a command"
        );
        assert!(failed.parked, "a failed world must still be left quiet");

        owner.close().await;
    }
}
