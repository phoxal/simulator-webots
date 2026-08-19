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
use phoxal::bus::WorldStepToken;
use phoxal::runtime::api::simulation::Clock;
use phoxal::simulator::WorldTime;

use crate::backend::Advance;
use crate::capabilities::SensorStep;

/// The world one turn of the step loop drives.
///
/// The loop is written against this rather than against
/// [`WebotsBackend`](crate::backend::WebotsBackend) directly for one reason: a
/// `WebotsBackend` exists only inside a live Webots controller process, while
/// what this loop guarantees - every output of a step published before the
/// clock that closes it, and a parked world however the loop stops - is exactly
/// what has to stay covered without one. There is a single production
/// implementation, and the tests below drive this seam against a real
/// simulator session.
pub(crate) trait StepWorld {
    /// Apply the graph's inputs and advance the world one step.
    fn advance(&mut self) -> Result<Advance>;

    /// Publish everything the completed step produced, stamped with its token.
    fn publish_due(&mut self, step: SensorStep, world_step: &WorldStepToken) -> Result<()>;

    /// Leave the world quiet.
    fn park(&mut self) -> Result<()>;
}

pub(crate) struct ControllerRuntime<W: StepWorld> {
    /// The world's own time: this process's exclusive timeline authority and
    /// the one clock hand that closes a step. It is the only way anything in
    /// this process can express a robot instant, and the only contract this
    /// controller publishes as itself rather than on behalf of a device - every
    /// capability's own handle lives with the device serving it.
    world_time: WorldTime,
    step_index: u64,
    world: W,
    /// Set by the host when a signal asks this controller to stop. The loop
    /// reads it between steps, so a stop takes effect at a step boundary and
    /// never inside one.
    stop: Arc<AtomicBool>,
}

impl<W: StepWorld> ControllerRuntime<W> {
    pub(crate) const fn new(world_time: WorldTime, world: W, stop: Arc<AtomicBool>) -> Self {
        Self {
            world_time,
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
            self.world_time.replace_timeline();
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
        let world_step = self.world_time.completed_step(time_ns);
        self.world
            .publish_due(SensorStep { time_ns }, &world_step)?;
        self.world_time
            .publish_clock(&world_step, Clock { step: next_step })?;
        self.step_index = next_step;
        tracing::trace!(
            target: crate::LOG_TARGET,
            timeline = %self.world_time.timeline(),
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
    use phoxal::api;
    use phoxal::bus::{CaptureStamp, RobotInstant, SamplePublisher, StepStamp};
    use phoxal::identity::TimelineId;
    use phoxal::model::identity::{CapabilityId, ComponentInstanceId};
    use phoxal::simulator::SimulatorSession;
    use std::collections::VecDeque;
    use std::sync::Mutex;

    /// The same diagnostic label the controller attaches with.
    const LABEL: &str = "webots-controller";

    /// A world scripted advance by advance, standing in for the Webots one.
    ///
    /// It publishes one encoder sample per step on a real handle taken from a
    /// real session, so what the loop commits is admitted by the transport the
    /// controller actually uses rather than recorded by a stub.
    struct ScriptedWorld {
        advances: VecDeque<Result<Advance>>,
        encoder: SamplePublisher<api::component::encoder::Sample>,
        /// The instant of every step whose outputs went out, in publish order.
        published_at: Arc<Mutex<Vec<RobotInstant>>>,
        /// Set once a step has published, standing in for a signal arriving
        /// while the loop is mid-step.
        stop_after_publish: Option<Arc<AtomicBool>>,
        /// Refuse this step's outputs, standing in for a device that failed
        /// halfway through a commit.
        refuse_outputs: bool,
        /// Observed after the runtime that owns this world has been consumed.
        parked: Arc<AtomicBool>,
    }

    impl StepWorld for ScriptedWorld {
        fn advance(&mut self) -> Result<Advance> {
            self.advances.pop_front().unwrap_or(Ok(Advance::Stopped))
        }

        fn publish_due(&mut self, step: SensorStep, world_step: &WorldStepToken) -> Result<()> {
            if self.refuse_outputs {
                anyhow::bail!("a capability refused this step's outputs");
            }
            self.encoder.publish(
                CaptureStamp::exact(world_step.instant()),
                api::component::encoder::Sample::try_new(step.time_ns as f64, 0.5)?,
            )?;
            self.published_at
                .lock()
                .expect("the publish record is uncontended")
                .push(world_step.instant());
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
    /// this process's one world time - has been dropped.
    struct Scenario {
        outcome: Result<()>,
        parked: bool,
        timeline: TimelineId,
        published_at: Vec<RobotInstant>,
    }

    /// Run one scripted world to completion against its own simulator session.
    ///
    /// The session is opened the way an adapter opens one, minus the router:
    /// the world time under test comes from `take_world_time`, exactly as it
    /// does in `controller::run`.
    async fn run_scenario(
        advances: Vec<Result<Advance>>,
        stop: &Arc<AtomicBool>,
        stop_after_publish: Option<Arc<AtomicBool>>,
        refuse_outputs: bool,
    ) -> Scenario {
        let mut session = SimulatorSession::in_process(LABEL)
            .await
            .expect("the in-process simulator session opens");
        let component = ComponentInstanceId::new("left_drive").expect("a valid component instance");
        let capability = CapabilityId::new("encoder").expect("a valid capability id");
        let encoder = session
            .sample_publisher(
                api::topics()
                    .component(&component)
                    .expect("a concrete component segment")
                    .encoder(&capability)
                    .expect("a concrete capability segment")
                    .sample()
                    .owner(),
            )
            .expect("the encoder publisher attaches");
        let world_time = session.take_world_time().expect("world time is available");
        let timeline = world_time.timeline();

        let parked = Arc::new(AtomicBool::new(false));
        let published_at = Arc::new(Mutex::new(Vec::new()));
        let world = ScriptedWorld {
            advances: advances.into_iter().collect(),
            encoder,
            published_at: Arc::clone(&published_at),
            stop_after_publish,
            refuse_outputs,
            parked: Arc::clone(&parked),
        };

        // `run` consumes the runtime, so the world time - and this process's one
        // timeline authority with it - is released before the next scenario
        // opens its session.
        let outcome = ControllerRuntime::new(world_time, world, Arc::clone(stop)).run();
        session
            .close()
            .await
            .expect("the in-process simulator session closes cleanly");

        Scenario {
            outcome,
            parked: parked.load(Ordering::Acquire),
            timeline,
            published_at: published_at
                .lock()
                .expect("the publish record is uncontended")
                .clone(),
        }
    }

    /// One completed advance, stopped after it.
    fn one_step() -> Vec<Result<Advance>> {
        vec![Ok(Advance::Stepped {
            time_ns: 20_000_000,
            rewound: false,
        })]
    }

    /// One process may mint exactly one world time, so every way the loop can
    /// stop is covered by this single test, each scenario closing its session -
    /// and with it the authority - before the next one opens. The session is
    /// the one an adapter really attaches with, minus the router, so a step's
    /// outputs are admitted by the transport the controller actually uses.
    ///
    /// What this proves is the parking discipline and the step stamping: every
    /// way the loop can stop leaves the world quiet, a device failure is
    /// reported rather than swallowed, and the outputs of an advance carry that
    /// advance's exact instant on this process's own timeline.
    ///
    /// What it deliberately does **not** claim is the enqueue order - a step's
    /// outputs reaching the outbound lane before the clock that closes it. That
    /// needs both sequence numbers read back from the client side, and the
    /// `simulator` profile hands an adapter owner-side handles only, by design.
    /// The framework proves it in `phoxal/src/simulator/world_session_tests.rs`,
    /// over the same transport, with a real subscriber on each key. Asserting it
    /// from here without that subscriber only looks like a proof: swapping the
    /// two publishes in `step_once` leaves every assertion below passing.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn every_stop_parks_the_world_and_a_step_stamps_its_outputs_with_its_own_instant() {
        // Webots quits the world after one step: the step is committed, and the
        // world is quiet before the loop returns.
        let stopped =
            run_scenario(one_step(), &Arc::new(AtomicBool::new(false)), None, false).await;
        assert!(stopped.outcome.is_ok(), "a quit world is not a failure");
        assert!(stopped.parked, "a stopped world must be left quiet");
        assert_eq!(
            stopped.published_at,
            vec![RobotInstant::new(stopped.timeline, 20_000_000)],
            "a step's outputs carry the instant the world advanced to, on the \
             timeline this process owns"
        );

        // A device that fails halfway through a commit is reported, and the
        // world is quiet all the same.
        let refused = run_scenario(one_step(), &Arc::new(AtomicBool::new(false)), None, true).await;
        assert_eq!(
            refused
                .outcome
                .expect_err("a refused output must reach the caller")
                .to_string(),
            "a capability refused this step's outputs"
        );
        assert!(
            refused.parked,
            "a world that failed mid-commit must still be left quiet"
        );

        // A host signal stops the loop at the next step boundary, before the
        // second scripted advance, and that world is left quiet too.
        let stop = Arc::new(AtomicBool::new(false));
        let mut signalled_advances = one_step();
        signalled_advances.push(Err(anyhow::anyhow!(
            "the loop must never reach this advance"
        )));
        let signalled =
            run_scenario(signalled_advances, &stop, Some(Arc::clone(&stop)), false).await;
        assert!(
            signalled.outcome.is_ok(),
            "a requested stop is not a failure"
        );
        assert!(signalled.parked, "a signalled world must be left quiet");
        assert_eq!(
            signalled.published_at,
            vec![RobotInstant::new(signalled.timeline, 20_000_000)],
            "the stop took effect at the step boundary, so only the first \
             advance ever published"
        );

        // A failing world is reported, and parked first: a motor left running
        // is worse than an error nobody sees.
        let failed = run_scenario(
            vec![Err(anyhow::anyhow!("device rejected a command"))],
            &Arc::new(AtomicBool::new(false)),
            None,
            false,
        )
        .await;
        assert_eq!(
            failed
                .outcome
                .expect_err("a device failure must reach the caller")
                .to_string(),
            "device rejected a command"
        );
        assert!(failed.parked, "a failed world must still be left quiet");
    }
}
