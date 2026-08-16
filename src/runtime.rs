//! The external Webots step loop.
//!
//! Webots owns the cadence, so this controller runs no framework step loop of
//! its own. Each iteration applies the actuator inputs, advances the world one
//! step, and commits everything that step produced.

use std::ops::ControlFlow;

use anyhow::Result;
use phoxal_bus::{TimelineAuthority, TimelineId};

use crate::backend::{Advance, SharedBackend};
use crate::controller::WorldClock;

pub(crate) struct ControllerRuntime {
    /// This controller's exclusive ownership of the world's timeline. It is
    /// the only way anything in this process can express a robot instant.
    authority: TimelineAuthority,
    step_index: u64,
    backend: SharedBackend,
}

impl ControllerRuntime {
    pub(crate) const fn new(authority: TimelineAuthority, backend: SharedBackend) -> Self {
        Self {
            authority,
            step_index: 0,
            backend,
        }
    }

    /// Step the world until it stops.
    ///
    /// Either way it stops - Webots asking the controller to shut down, or a
    /// device failing - the world is left quiet before this returns, so a
    /// stopped loop never leaves the motors running. Only the second is an
    /// error: a reverted or quit world is how a simulation ends.
    pub(crate) async fn run(mut self, clock: WorldClock) -> Result<()> {
        loop {
            match self.step_once(&clock).await {
                Ok(ControlFlow::Continue(())) => {}
                Ok(ControlFlow::Break(())) => {
                    tracing::info!(
                        target: crate::LOG_TARGET,
                        steps = self.step_index,
                        "Webots asked the controller to shut down"
                    );
                    return self.backend.park().await;
                }
                Err(error) => {
                    if let Err(park_error) = self.backend.park().await {
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

    async fn step_once(&mut self, clock: &WorldClock) -> Result<ControlFlow<()>> {
        let Advance::Stepped {
            time_ns,
            output,
            rewound,
        } = self.backend.advance().await?
        else {
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
        clock.commit_step(&world_step, next_step, output)?;
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
