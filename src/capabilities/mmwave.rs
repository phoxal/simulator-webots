//! mmWave capability: publishes `component::mmwave::Scan` from the Webots
//! `Radar` device.
//!
//! Webots reports each target as range, azimuth, radial speed, and received
//! power. The contract wants cartesian position and velocity, so the bearing is
//! resolved here; elevation is zero because a Webots radar target carries none.

use anyhow::Result;
use phoxal::api;
use webots_rs::device::radar::RadarTarget;

use super::{SampledSpec, SensorStep, SimulatedSensor};

pub(crate) struct NativeMmwave {
    radar: webots_rs::device::radar::Radar,
    spec: SampledSpec,
}

impl NativeMmwave {
    pub(crate) fn new(webots: &webots_rs::Webots, spec: &SampledSpec) -> Result<Self> {
        let radar = webots.radar(spec.reference.to_string())?;
        radar.enable(spec.sampling_period_ms)?;
        Ok(Self {
            radar,
            spec: spec.clone(),
        })
    }
}

impl SimulatedSensor for NativeMmwave {
    type Sample = api::component::mmwave::Scan;

    fn schedule(&mut self) -> &mut phoxal::SampleSchedule {
        &mut self.spec.schedule
    }

    fn read(&mut self, _step: SensorStep) -> Result<Option<Self::Sample>> {
        let targets = self.radar.targets()?;
        api::component::mmwave::Scan::try_new(targets.iter().map(detection).collect())
            .map(Some)
            .map_err(anyhow::Error::from)
    }
}

/// One Webots radar target as a typed detection.
///
/// `snr` carries the target's received power. Webots models no noise floor, so
/// there is no true signal-to-noise ratio to report and inventing one would be
/// worse than passing through the only strength figure the simulator has.
fn detection(target: &RadarTarget) -> api::component::mmwave::Detection {
    let (sin_azimuth, cos_azimuth) = target.azimuth.sin_cos();
    api::component::mmwave::Detection {
        position: [
            (target.distance * cos_azimuth) as f32,
            (target.distance * sin_azimuth) as f32,
            0.0,
        ],
        velocity: [
            (target.speed * cos_azimuth) as f32,
            (target.speed * sin_azimuth) as f32,
            0.0,
        ],
        snr: target.received_power as f32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_target_dead_ahead_lies_on_the_x_axis() {
        let detection = detection(&RadarTarget {
            distance: 4.0,
            received_power: -20.0,
            speed: 1.5,
            azimuth: 0.0,
        });
        assert_eq!(detection.position, [4.0, 0.0, 0.0]);
        assert_eq!(detection.velocity, [1.5, 0.0, 0.0]);
        assert_eq!(detection.snr, -20.0);
    }

    #[test]
    fn azimuth_rotates_range_and_radial_speed_together() {
        let detection = detection(&RadarTarget {
            distance: 2.0,
            received_power: 0.0,
            speed: 2.0,
            azimuth: std::f64::consts::FRAC_PI_2,
        });
        assert!(detection.position[0].abs() < 1e-6);
        assert!((detection.position[1] - 2.0).abs() < 1e-6);
        assert!((detection.velocity[1] - 2.0).abs() < 1e-6);
    }
}
