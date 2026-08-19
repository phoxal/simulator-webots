//! Encoder capability: publishes `component::encoder::Sample` from the Webots
//! `PositionSensor` device.
//!
//! Webots reports the joint's own angle. The contract carries the actuator's,
//! so both position and the velocity differentiated from it are scaled by the
//! declared gear ratio.

use anyhow::Result;
use phoxal::api;

use super::{SampledSpec, SensorStep, SimulatedSensor};

#[derive(Clone, Debug)]
pub(crate) struct EncoderSpec {
    pub(crate) sampled: SampledSpec,
    pub(crate) gear_ratio: f64,
}

pub(crate) struct NativeEncoder {
    sensor: webots_rs::device::position_sensor::PositionSensor,
    spec: EncoderSpec,
    last: Option<(f64, u64)>,
}

impl NativeEncoder {
    pub(crate) fn new(webots: &webots_rs::Webots, spec: &EncoderSpec) -> Result<Self> {
        let sensor = webots.position_sensor(spec.sampled.reference.to_string())?;
        sensor.enable(spec.sampled.sampling_period_ms)?;
        Ok(Self {
            sensor,
            spec: spec.clone(),
            last: None,
        })
    }
}

impl SimulatedSensor for NativeEncoder {
    type Sample = api::component::encoder::Sample;

    fn schedule(&mut self) -> &mut phoxal::SampleSchedule {
        &mut self.spec.sampled.schedule
    }

    fn reset(&mut self, logical_time_ns: u64) -> Result<()> {
        let delay_ns = self.spec.sampled.schedule.period_ns();
        self.spec
            .sampled
            .schedule
            .reanchor_after(logical_time_ns, delay_ns)?;
        self.last = None;
        Ok(())
    }

    /// Webots exposes no joint velocity, so it is differentiated across the
    /// window between two published readings. The first reading has no window
    /// to differentiate over and reports zero.
    fn read(&mut self, step: SensorStep) -> Result<Option<Self::Sample>> {
        let position_rad = joint_to_actuator_position(self.sensor.value()?, self.spec.gear_ratio);
        let velocity_radps = self
            .last
            .map(|(last_position, last_time)| {
                velocity_radps(position_rad, last_position, step.time_ns, last_time)
            })
            .unwrap_or(0.0);
        self.last = Some((position_rad, step.time_ns));
        api::component::encoder::Sample::try_new(position_rad, velocity_radps as f32)
            .map(Some)
            .map_err(anyhow::Error::from)
    }
}

fn joint_to_actuator_position(joint_position_rad: f64, gear_ratio: f64) -> f64 {
    joint_position_rad * gear_ratio
}

fn velocity_radps(
    position: f64,
    previous_position: f64,
    time_ns: u64,
    previous_time_ns: u64,
) -> f64 {
    let dt_ns = time_ns.saturating_sub(previous_time_ns);
    if dt_ns == 0 {
        0.0
    } else {
        (position - previous_position) * 1_000_000_000.0 / dt_ns as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joint_to_actuator_scales_by_gear_ratio() {
        assert_eq!(joint_to_actuator_position(2.0, 2.0), 4.0);
    }

    #[test]
    fn velocity_is_position_delta_over_time_delta() {
        assert_eq!(velocity_radps(2.0, 1.0, 2_000_000_000, 1_000_000_000), 1.0);
    }
}
