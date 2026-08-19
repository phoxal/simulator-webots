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
/// what has to stay coverable without one. There is a single production
/// implementation.
///
/// That cover is currently owed rather than held: the loop's other half is a
/// [`WorldTime`], which only a connected [`SimulatorSession`] can hand out, and
/// the `simulator` profile offers no session that opens without a live router.
/// The seam stays because the test comes straight back the moment it does.
///
/// [`SimulatorSession`]: phoxal::simulator::SimulatorSession
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
