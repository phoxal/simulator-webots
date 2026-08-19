//! Camera capability: publishes `component::camera::Frame` from the Webots
//! `Camera` device.
//!
//! Webots always hands back BGRA. The component's declared mode decides which
//! contract encoding the frame carries, so the conversion to RGB or to
//! luminance happens here rather than in every consumer.

use anyhow::Result;
use phoxal::api;
use phoxal::model::component::capability::CameraMode;

use super::{SampledSpec, SensorStep, SimulatedSensor};

#[derive(Clone, Debug)]
pub(crate) struct CameraSpec {
    pub(crate) sampled: SampledSpec,
    pub(crate) mode: CameraMode,
    pub(crate) width: u32,
    pub(crate) height: u32,
}

pub(crate) struct NativeCamera {
    camera: webots_rs::device::camera::Camera,
    spec: CameraSpec,
}

impl NativeCamera {
    pub(crate) fn new(webots: &webots_rs::Webots, spec: &CameraSpec) -> Result<Self> {
        let camera = webots.camera(spec.sampled.reference.to_string())?;
        camera.enable(spec.sampled.sampling_period_ms)?;
        Ok(Self {
            camera,
            spec: spec.clone(),
        })
    }
}

impl SimulatedSensor for NativeCamera {
    type Sample = api::component::camera::Frame;

    fn schedule(&mut self) -> &mut phoxal::SampleSchedule {
        &mut self.spec.sampled.schedule
    }

    fn read(&mut self, _step: SensorStep) -> Result<Option<Self::Sample>> {
        let bgra = self.camera.get_image()?;
        let (encoding, data) = match self.spec.mode {
            CameraMode::Mono => (api::component::camera::Encoding::L8, bgra_to_luma(&bgra)),
            CameraMode::Rgb => (api::component::camera::Encoding::Rgb8, bgra_to_rgb(&bgra)),
        };
        Ok(Some(api::component::camera::Frame::try_new(
            self.spec.width,
            self.spec.height,
            encoding,
            None,
            None,
            None,
            None,
            data,
        )?))
    }
}

fn bgra_to_rgb(bgra: &[u8]) -> Vec<u8> {
    bgra.chunks_exact(4)
        .flat_map(|pixel| [pixel[2], pixel[1], pixel[0]])
        .collect()
}

fn bgra_to_luma(bgra: &[u8]) -> Vec<u8> {
    bgra.chunks_exact(4)
        .map(|pixel| {
            let red = u32::from(pixel[2]);
            let green = u32::from(pixel[1]);
            let blue = u32::from(pixel[0]);
            ((299 * red + 587 * green + 114 * blue) / 1000) as u8
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bgra_to_rgb_swaps_channel_order() {
        assert_eq!(bgra_to_rgb(&[10, 20, 30, 255]), vec![30, 20, 10]);
    }

    #[test]
    fn bgra_to_luma_applies_bt601_weights() {
        assert_eq!(bgra_to_luma(&[10, 20, 30, 255]), vec![21]);
    }
}
