//! The Webots controller-process handle and the world it steps.
//!
//! This crate never opens a `webots_rs::Supervisor`: a controller drives its
//! own robot's devices and nothing else in the world.

use anyhow::{Context, Result, bail};
use phoxal::bus::WorldStepToken;

use crate::capabilities::SensorStep;
use crate::channel::CapabilityChannel;
use crate::runtime::StepWorld;

/// What one turn of the Webots step loop produced.
pub(crate) enum Advance {
    /// The world advanced one step, reaching `time_ns`. `rewound` reports that
    /// this instant precedes the previous one, which is Webots starting a new
    /// world history rather than continuing this one.
    Stepped { time_ns: u64, rewound: bool },
    /// Webots asked this controller to shut down - the world was reverted,
    /// reloaded, or quit. That is how a simulation ends, not how one fails, so
    /// it is a value here rather than an error.
    Stopped,
}

/// The open controller-process handle, before any capability is bound to it.
///
/// Opening is separate from binding because a device can only be requested
/// from an already-open handle, while the bus side of the same capability can
/// only be attached from an open simulator session. Both happen in one pass
/// over the catalog, and this is the handle that pass borrows.
pub(crate) struct WebotsHandle {
    webots: webots_rs::Webots,
    step_ms: i32,
}

impl WebotsHandle {
    /// Open this process's Webots controller handle and resolve the world's
    /// step length once.
    ///
    /// # Errors
    ///
    /// Returns an error when Webots refuses the handle, or when the world's
    /// `basicTimeStep` describes no advance at all.
    pub(crate) fn open() -> Result<Self> {
        let webots = webots_rs::Webots::new()?;
        let step_ms = webots.get_basic_time_step()?.round() as i32;
        step_ns_from_ms(step_ms)?;
        Ok(Self { webots, step_ms })
    }

    /// The handle capability devices are requested from.
    pub(crate) const fn webots(&self) -> &webots_rs::Webots {
        &self.webots
    }

    /// The world's actual integer control step, obtained from Webots rather
    /// than assumed by the catalog.
    pub(crate) const fn basic_time_step_ms(&self) -> i32 {
        self.step_ms
    }

    /// Take ownership of the world now that every capability is bound to it.
    pub(crate) fn into_backend(self, channels: Vec<CapabilityChannel>) -> WebotsBackend {
        WebotsBackend {
            webots: self.webots,
            step_ms: self.step_ms,
            last_time_ns: None,
            channels,
        }
    }
}

/// The world, its step length, and every capability bound to it.
///
/// One dedicated OS thread owns this for the life of the process, so every
/// Webots call is made from the thread that opened the devices and a step
/// publishes what it read without handing anything back across a task boundary.
pub(crate) struct WebotsBackend {
    webots: webots_rs::Webots,
    step_ms: i32,
    last_time_ns: Option<u64>,
    channels: Vec<CapabilityChannel>,
}

impl StepWorld for WebotsBackend {
    /// Apply the graph's inputs and advance the world one step.
    ///
    /// # Errors
    ///
    /// Returns an error when a device rejects a command or Webots reports an
    /// unusable clock. Webots asking the controller to shut down is
    /// [`Advance::Stopped`], not an error: a reverted or quit world ends the
    /// simulation normally.
    fn advance(&mut self) -> Result<Advance> {
        for channel in &mut self.channels {
            channel.apply_backlog()?;
        }
        if !self.webots.step(self.step_ms)? {
            return Ok(Advance::Stopped);
        }
        let time_ns = simulation_time_ns(self.webots.get_time()?)?;
        let rewound = time_was_rewound(self.last_time_ns, time_ns);
        if rewound {
            for channel in &mut self.channels {
                channel.reset(time_ns)?;
            }
        }
        self.last_time_ns = Some(time_ns);
        Ok(Advance::Stepped { time_ns, rewound })
    }

    /// Read every capability that publishes on the step just completed and
    /// publish it, each on its own handle, in the robot's binding order.
    fn publish_due(&mut self, step: SensorStep, world_step: &WorldStepToken) -> Result<()> {
        for channel in &mut self.channels {
            channel.publish_due(step, world_step)?;
        }
        Ok(())
    }

    /// Leave the world quiet: every motor stopped and every speaker silent.
    /// A simulation that stopped must not keep driving or keep playing.
    ///
    /// Every capability is parked even after one of them fails, because a
    /// motor left running is worse than an unreported error; the first failure
    /// is what the caller sees.
    fn park(&mut self) -> Result<()> {
        let mut first_error = None;
        for channel in &mut self.channels {
            if let Err(error) = channel.park() {
                tracing::warn!(
                    target: crate::LOG_TARGET,
                    capability = %channel.reference(),
                    error = %error,
                    "failed to quiet a capability while stopping"
                );
                first_error.get_or_insert(error);
            }
        }
        first_error.map_or(Ok(()), Err)
    }
}

/// The world's step length in nanoseconds. Webots reports whole milliseconds,
/// and a non-positive or overflowing step describes no advance at all.
fn step_ns_from_ms(step_ms: i32) -> Result<u64> {
    let step_ms =
        u64::try_from(step_ms).context("Webots basicTimeStep must be a positive integer")?;
    if step_ms == 0 {
        bail!("Webots basicTimeStep must be > 0");
    }
    step_ms
        .checked_mul(1_000_000)
        .context("Webots basicTimeStep overflows nanoseconds")
}

/// Convert Webots' seconds clock to the framework's checked nanosecond domain.
fn simulation_time_ns(seconds: f64) -> Result<u64> {
    if !seconds.is_finite() || seconds < 0.0 {
        bail!("Webots simulation time must be finite and >= 0");
    }
    let nanos = seconds * 1_000_000_000.0;
    if !nanos.is_finite() || !(0.0..18_446_744_073_709_551_616.0).contains(&nanos) {
        bail!("Webots simulation time overflows nanoseconds");
    }
    Ok(nanos.round() as u64)
}

fn time_was_rewound(previous_time_ns: Option<u64>, time_ns: u64) -> bool {
    previous_time_ns.is_some_and(|previous_time_ns| time_ns < previous_time_ns)
}

#[cfg(test)]
mod tests {
    use super::{simulation_time_ns, step_ns_from_ms, time_was_rewound};

    #[test]
    fn webots_step_duration_must_be_positive() {
        assert_eq!(step_ns_from_ms(10).expect("10 ms is valid"), 10_000_000);
        assert!(step_ns_from_ms(0).is_err());
        assert!(step_ns_from_ms(-1).is_err());
    }

    #[test]
    fn webots_time_is_checked_when_converted_to_nanoseconds() {
        assert_eq!(simulation_time_ns(0.125).unwrap(), 125_000_000);
        assert!(simulation_time_ns(-1.0).is_err());
        assert!(simulation_time_ns(f64::NAN).is_err());
        assert!(simulation_time_ns(f64::INFINITY).is_err());
    }

    #[test]
    fn a_webots_rewind_is_detected_before_sensor_reads() {
        assert!(!time_was_rewound(None, 0));
        assert!(!time_was_rewound(Some(10_000_000), 20_000_000));
        assert!(time_was_rewound(Some(20_000_000), 10_000_000));
    }
}
