//! IMU capability: publishes `component::imu::Sample` from the Webots
//! `InertialUnit`, `Accelerometer` and `Gyro` devices.
//!
//! Webots has no single inertial device, so one declared IMU binds three world
//! nodes. The two extra nodes are named by suffixing the capability, which is
//! the naming a simulated component must use for its IMU to bind at all.

use anyhow::Result;
use phoxal_protocol::robot as api;

use super::{SampledSpec, SensorStep, SimulatedSensor};

pub(crate) struct NativeImu {
    inertial_unit: webots_rs::device::inertial_unit::InertialUnit,
    accelerometer: webots_rs::device::accelerometer::Accelerometer,
    gyro: webots_rs::device::gyro::Gyro,
    spec: SampledSpec,
}

impl NativeImu {
    pub(crate) fn new(webots: &webots_rs::Webots, spec: &SampledSpec) -> Result<Self> {
        let inertial_unit = webots.inertial_unit(spec.reference.to_string())?;
        let accelerometer = webots.accelerometer(format!("{}__accel", spec.reference))?;
        let gyro = webots.gyro(format!("{}__gyro", spec.reference))?;
        inertial_unit.enable(spec.sampling_period_ms)?;
        accelerometer.enable(spec.sampling_period_ms)?;
        gyro.enable(spec.sampling_period_ms)?;
        Ok(Self {
            inertial_unit,
            accelerometer,
            gyro,
            spec: spec.clone(),
        })
    }
}

impl SimulatedSensor for NativeImu {
    type Sample = api::component::imu::Sample;
    type Endpoint = api::endpoint::component::imu::SampleEndpoint;

    fn schedule(&mut self) -> &mut crate::sample_schedule::SampleSchedule {
        &mut self.spec.schedule
    }

    fn read(&mut self, _step: SensorStep) -> Result<Option<Self::Sample>> {
        let [roll, pitch, yaw] = self.inertial_unit.get_roll_pitch_yaw()?;
        let acceleration = self.accelerometer.values()?.map(|value| value as f32);
        let angular_velocity = self.gyro.values()?.map(|value| value as f32);
        api::component::imu::Sample::try_new(
            Some(quaternion_wxyz_from_rpy(roll, pitch, yaw)),
            angular_velocity,
            acceleration,
            None,
            None,
            None,
            api::component::imu::SensorHealth::Nominal,
            None,
        )
        .map(Some)
        .map_err(anyhow::Error::from)
    }
}

fn quaternion_wxyz_from_rpy(roll: f64, pitch: f64, yaw: f64) -> [f32; 4] {
    let half_roll = roll * 0.5;
    let half_pitch = pitch * 0.5;
    let half_yaw = yaw * 0.5;
    let (sr, cr) = half_roll.sin_cos();
    let (sp, cp) = half_pitch.sin_cos();
    let (sy, cy) = half_yaw.sin_cos();

    [
        (cr * cp * cy + sr * sp * sy) as f32,
        (sr * cp * cy - cr * sp * sy) as f32,
        (cr * sp * cy + sr * cp * sy) as f32,
        (cr * cp * sy - sr * sp * cy) as f32,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quaternion_from_yaw_is_wxyz() {
        let quaternion = quaternion_wxyz_from_rpy(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let half = (std::f64::consts::FRAC_PI_2 * 0.5).sin_cos();
        assert!((f64::from(quaternion[0]) - half.1).abs() < 1e-6);
        assert!(f64::from(quaternion[1]).abs() < 1e-6);
        assert!(f64::from(quaternion[2]).abs() < 1e-6);
        assert!((f64::from(quaternion[3]) - half.0).abs() < 1e-6);
    }
}
