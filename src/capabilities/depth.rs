//! Depth capability: publishes `component::depth::Frame` from the Webots
//! `RangeFinder` device.
//!
//! Webots reports metres as floats; the contract carries unsigned millimetres
//! with zero reserved for "no return", so the conversion below is where a
//! non-finite or non-positive reading becomes that reserved value rather than
//! a plausible-looking distance.

use anyhow::Result;
use phoxal_protocol::robot as api;

use super::{SampledSpec, SensorStep, SimulatedSensor};

#[derive(Clone, Debug)]
pub(crate) struct DepthSpec {
    pub(crate) sampled: SampledSpec,
    pub(crate) width: u32,
    pub(crate) height: u32,
}

pub(crate) struct NativeDepth {
    sensor: webots_rs::device::range_finder::RangeFinder,
    spec: DepthSpec,
}

impl NativeDepth {
    pub(crate) fn new(webots: &webots_rs::Webots, spec: &DepthSpec) -> Result<Self> {
        let sensor = webots.range_finder(spec.sampled.reference.to_string())?;
        sensor.enable(spec.sampled.sampling_period_ms)?;
        Ok(Self {
            sensor,
            spec: spec.clone(),
        })
    }
}

impl SimulatedSensor for NativeDepth {
    type Sample = api::component::depth::Frame;
    type Endpoint = api::endpoint::component::depth::FrameEndpoint;

    fn schedule(&mut self) -> &mut crate::sample_schedule::SampleSchedule {
        &mut self.spec.sampled.schedule
    }

    fn read(&mut self, _step: SensorStep) -> Result<Option<Self::Sample>> {
        let samples_mm = self
            .sensor
            .get_range_image()?
            .into_iter()
            .map(meters_to_u16_mm)
            .collect();
        api::component::depth::Frame::try_new(
            samples_mm,
            api::component::depth::Encoding::U16Millimeters,
            api::component::depth::InvalidSamplePolicy::ZeroIsInvalid,
            self.spec.width,
            self.spec.height,
            None,
            None,
            None,
            None,
        )
        .map(Some)
        .map_err(anyhow::Error::from)
    }
}

fn meters_to_u16_mm(meters: f32) -> u16 {
    if !meters.is_finite() || meters <= 0.0 {
        return 0;
    }
    let millimeters = (meters * 1000.0).round();
    if !(1.0..=f32::from(u16::MAX)).contains(&millimeters) {
        return 0;
    }
    millimeters as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn meters_to_u16_mm_rounds_and_marks_unrepresentable_values_invalid() {
        assert_eq!(meters_to_u16_mm(1.25), 1250);
        assert_eq!(meters_to_u16_mm(f32::NAN), 0);
        assert_eq!(meters_to_u16_mm(0.0001), 0);
        assert_eq!(meters_to_u16_mm(70.0), 0);
    }
}
