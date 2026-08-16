//! Accelerometer capability: publishes `component::accelerometer::Sample` from
//! the Webots `Accelerometer` device, which reports proper acceleration in the
//! sensor's own frame.

use phoxal_protocol::robot as api;

use super::vector_sensor;

vector_sensor!(
    NativeAccelerometer,
    webots_rs::device::accelerometer::Accelerometer,
    accelerometer,
    api::component::accelerometer::Sample,
    api::endpoint::component::accelerometer::SampleEndpoint,
    linear_acceleration,
);
