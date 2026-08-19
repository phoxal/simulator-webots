//! Gyroscope capability: publishes `component::gyroscope::Sample` from the
//! Webots `Gyro` device, which reports angular velocity about the sensor's own
//! axes.

use phoxal::api;

use super::vector_sensor;

vector_sensor!(
    NativeGyroscope,
    webots_rs::device::gyro::Gyro,
    gyro,
    api::component::gyroscope::Sample,
    angular_velocity,
);
