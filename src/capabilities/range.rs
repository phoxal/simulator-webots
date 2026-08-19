//! Range capability: publishes `component::range::Sample` from the Webots
//! `DistanceSensor` device, bounded by the range limits the component
//! declares.

use anyhow::Result;
use phoxal::api;

use super::{SampledSpec, SensorStep, SimulatedSensor};

#[derive(Clone, Debug)]
pub(crate) struct RangeSpec {
    pub(crate) sampled: SampledSpec,
    pub(crate) min_range_m: f32,
    pub(crate) max_range_m: f32,
}

pub(crate) struct NativeRange {
    sensor: webots_rs::device::distance_sensor::DistanceSensor,
    spec: RangeSpec,
}

impl NativeRange {
    pub(crate) fn new(webots: &webots_rs::Webots, spec: &RangeSpec) -> Result<Self> {
        let sensor = webots.distance_sensor(spec.sampled.reference.to_string())?;
        sensor.enable(spec.sampled.sampling_period_ms)?;
        Ok(Self {
            sensor,
            spec: spec.clone(),
        })
    }
}

impl SimulatedSensor for NativeRange {
    type Sample = api::component::range::Sample;

    fn schedule(&mut self) -> &mut phoxal::SampleSchedule {
        &mut self.spec.sampled.schedule
    }

    fn read(&mut self, _step: SensorStep) -> Result<Option<Self::Sample>> {
        api::component::range::Sample::try_new(
            self.sensor.value()? as f32,
            Some(api::component::range::Limits {
                min_m: self.spec.min_range_m,
                max_m: self.spec.max_range_m,
            }),
            Some(api::component::range::SampleQuality {
                valid: true,
                confidence: None,
            }),
            api::component::range::SensorHealth::Nominal,
        )
        .map(Some)
        .map_err(anyhow::Error::from)
    }
}
