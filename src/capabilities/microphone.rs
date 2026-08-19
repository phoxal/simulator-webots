//! Microphone capability: publishes `component::microphone::Frame` from the
//! Webots `Microphone` device. Webots hands back the raw encoded sample block
//! for the elapsed window, so the frame carries it through untouched.

use anyhow::Result;
use phoxal::api;

use super::{SampledSpec, SensorStep, SimulatedSensor};

pub(crate) struct NativeMicrophone {
    microphone: webots_rs::device::microphone::Microphone,
    spec: SampledSpec,
}

impl NativeMicrophone {
    pub(crate) fn new(webots: &webots_rs::Webots, spec: &SampledSpec) -> Result<Self> {
        let microphone = webots.microphone(spec.reference.to_string())?;
        microphone.enable(spec.sampling_period_ms)?;
        Ok(Self {
            microphone,
            spec: spec.clone(),
        })
    }
}

impl SimulatedSensor for NativeMicrophone {
    type Sample = api::component::microphone::Frame;

    fn schedule(&mut self) -> &mut phoxal::SampleSchedule {
        &mut self.spec.schedule
    }

    fn read(&mut self, _step: SensorStep) -> Result<Option<Self::Sample>> {
        let data = self.microphone.get_sample_data()?;
        // A silent window yields no samples; publishing an empty frame would
        // claim an observation the sensor did not make.
        if data.is_empty() {
            return Ok(None);
        }
        Ok(Some(api::component::microphone::Frame { data }))
    }
}
