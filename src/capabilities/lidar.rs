//! Lidar capability: publishes `component::lidar::Scan` from the Webots
//! `Lidar` device, as polar ranges or as a cartesian point cloud depending on
//! the component's declared `output`.
//!
//! Geometry and range limits are read off the device rather than the manifest:
//! the world's `Lidar` node is what actually decides them, so reporting the
//! authored numbers instead would let a scan describe a sensor nobody built.

use anyhow::Result;
use phoxal::api;
use phoxal::model::component::capability::LidarOutput;
use webots_rs::device::lidar::{LidarConfig, LidarReading};

use super::{SampledSpec, SensorStep, SimulatedSensor};

#[derive(Clone, Debug)]
pub(crate) struct LidarSpec {
    pub(crate) sampled: SampledSpec,
    pub(crate) output: LidarOutput,
}

pub(crate) struct NativeLidar {
    lidar: webots_rs::device::lidar::Lidar,
    spec: LidarSpec,
    geometry: Option<api::component::lidar::ScanGeometry>,
    limits: api::component::lidar::RangeLimits,
}

impl NativeLidar {
    pub(crate) fn new(webots: &webots_rs::Webots, spec: &LidarSpec) -> Result<Self> {
        let point_cloud = matches!(spec.output, LidarOutput::Points);
        let lidar = webots.lidar(
            spec.sampled.reference.to_string(),
            LidarConfig::new().with_point_cloud(point_cloud),
        )?;
        lidar.enable(spec.sampled.sampling_period_ms)?;
        let limits = api::component::lidar::RangeLimits {
            min_m: lidar.get_min_range()? as f32,
            max_m: lidar.get_max_range()? as f32,
        };
        let geometry = scan_geometry(lidar.get_fov()?, lidar.get_horizontal_resolution()?);
        Ok(Self {
            lidar,
            spec: spec.clone(),
            geometry,
            limits,
        })
    }
}

impl SimulatedSensor for NativeLidar {
    type Sample = api::component::lidar::Scan;

    fn schedule(&mut self) -> &mut phoxal::SampleSchedule {
        &mut self.spec.sampled.schedule
    }

    fn read(&mut self, _step: SensorStep) -> Result<Option<Self::Sample>> {
        match self.lidar.reading()? {
            LidarReading::RangeImage(ranges) => {
                let ranges: Vec<_> = ranges
                    .into_iter()
                    .map(|range| {
                        if range.is_finite() {
                            api::component::lidar::RangeSample::Valid(range)
                        } else {
                            api::component::lidar::RangeSample::Invalid
                        }
                    })
                    .collect();
                let valid_points = ranges
                    .iter()
                    .filter(|range| matches!(range, api::component::lidar::RangeSample::Valid(_)))
                    .count();
                api::component::lidar::Scan::ranges(
                    ranges,
                    self.geometry,
                    Some(self.limits),
                    Some(api::component::lidar::ScanQuality {
                        valid_points: valid_points as u32,
                    }),
                    api::component::lidar::SensorHealth::Nominal,
                )
            }
            LidarReading::PointCloud(cloud) => {
                let points: Vec<_> = cloud
                    .iter()
                    .map(|point| {
                        let point = [point.x, point.y, point.z];
                        if point.iter().all(|axis| axis.is_finite()) {
                            api::component::lidar::PointSample::Valid(point)
                        } else {
                            api::component::lidar::PointSample::Invalid
                        }
                    })
                    .collect();
                let valid_points = points
                    .iter()
                    .filter(|point| matches!(point, api::component::lidar::PointSample::Valid(_)))
                    .count();
                api::component::lidar::Scan::points(
                    points,
                    Some(self.limits),
                    Some(api::component::lidar::ScanQuality {
                        valid_points: valid_points as u32,
                    }),
                    api::component::lidar::SensorHealth::Nominal,
                )
            }
        }
        .map(Some)
        .map_err(anyhow::Error::from)
    }
}

/// The polar geometry of one horizontal sweep. A single-ray lidar has no
/// increment to report, and a non-finite field of view describes no sweep at
/// all, so both yield `None` rather than an invented angle.
fn scan_geometry(
    fov_rad: f64,
    horizontal_resolution: i32,
) -> Option<api::component::lidar::ScanGeometry> {
    if !fov_rad.is_finite() || fov_rad <= 0.0 || horizontal_resolution < 2 {
        return None;
    }
    Some(api::component::lidar::ScanGeometry {
        angle_min_rad: (-fov_rad / 2.0) as f32,
        angle_increment_rad: (fov_rad / f64::from(horizontal_resolution - 1)) as f32,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometry_spreads_the_field_of_view_across_the_rays() {
        let geometry = scan_geometry(std::f64::consts::PI, 3).expect("a 3-ray sweep has geometry");
        assert_eq!(geometry.angle_min_rad, -(std::f64::consts::PI / 2.0) as f32);
        assert_eq!(
            geometry.angle_increment_rad,
            (std::f64::consts::PI / 2.0) as f32
        );
    }

    #[test]
    fn a_single_ray_or_absent_sweep_reports_no_geometry() {
        assert!(scan_geometry(std::f64::consts::PI, 1).is_none());
        assert!(scan_geometry(0.0, 8).is_none());
        assert!(scan_geometry(f64::NAN, 8).is_none());
    }
}
