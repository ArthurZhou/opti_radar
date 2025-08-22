// src/data_generator.rs

use crate::target_processor::Measurement;
use nalgebra::{Point3, Vector3};
use rand::prelude::*;
use std::f64::consts::PI;

/// Generates simulated radar measurement data and true target positions.
///
/// This function creates a set of measurements for multiple targets, including
/// realistic noise in station positions and measurement angles.
///
/// # Arguments
/// * `num_targets` - The number of targets to generate.
/// * `target_x_range`, `target_y_range`, `target_z_range` - The min/max ranges for target positions.
/// * `num_stations_per_target_range` - A tuple of min/max number of stations per target.
/// * `station_dist_range` - The min/max distance of stations from their respective targets.
/// * `station_z_range` - The min/max altitude range for stations.
/// * `pos_noise_std` - Standard deviation for station position noise.
/// * `alt_noise_std` - Standard deviation for station altitude noise.
/// * `angle_noise_std` - Standard deviation for measurement angle noise.
///
/// # Returns
/// A tuple containing:
/// * `Vec<Point3<f64>>` - A vector of the true, noise-free positions of the targets.
/// * `Vec<Measurement>` - A vector of the generated noisy measurements.
pub fn generate_data(
    num_targets: usize,
    target_x_range: (f64, f64),
    target_y_range: (f64, f64),
    target_z_range: (f64, f64),
    num_stations_per_target_range: (usize, usize),
    station_dist_range: (f64, f64),
    station_z_range: (f64, f64),
    pos_noise_std: f64,
    alt_noise_std: f64,
    angle_noise_std: f64,
) -> (Vec<Point3<f64>>, Vec<Measurement>) {
    let mut rng = thread_rng();
    let mut all_data = Vec::new();
    let mut true_targets = Vec::new();

    for _ in 0..num_targets {
        // Generate target position directly in Cartesian coordinates
        let true_target_pos = Point3::new(
            rng.gen_range(target_x_range.0..target_x_range.1),
            rng.gen_range(target_y_range.0..target_y_range.1),
            rng.gen_range(target_z_range.0..target_z_range.1),
        );
        true_targets.push(true_target_pos);

        let num_stations =
            rng.gen_range(num_stations_per_target_range.0..=num_stations_per_target_range.1);
        for _ in 0..num_stations {
            // Generate station position directly in Cartesian coordinates
            let angle = rng.gen_range(0.0..2.0 * PI);
            let dist = rng.gen_range(station_dist_range.0..station_dist_range.1);
            let true_station_pos = Point3::new(
                true_target_pos.x + dist * angle.cos(),
                true_target_pos.y + dist * angle.sin(),
                rng.gen_range(station_z_range.0..station_z_range.1),
            );

            let true_direction = (true_target_pos - true_station_pos).normalize();

            // Add noise
            let measured_station_pos = Point3::new(
                true_station_pos.x + rng.gen_range(-pos_noise_std..pos_noise_std),
                true_station_pos.y + rng.gen_range(-pos_noise_std..pos_noise_std),
                true_station_pos.z + rng.gen_range(-alt_noise_std..alt_noise_std),
            );

            let measured_direction = Vector3::new(
                true_direction.x + rng.gen_range(-angle_noise_std..angle_noise_std),
                true_direction.y + rng.gen_range(-angle_noise_std..angle_noise_std),
                true_direction.z + rng.gen_range(-angle_noise_std..angle_noise_std),
            )
            .normalize();

            all_data.push(Measurement {
                x: measured_station_pos.x,
                y: measured_station_pos.y,
                z: measured_station_pos.z,
                direction_x: measured_direction.x,
                direction_y: measured_direction.y,
                direction_z: measured_direction.z,
            });
        }
    }
    (true_targets, all_data)
}
