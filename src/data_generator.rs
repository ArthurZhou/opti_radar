// src/data_generator.rs

use crate::target_processor::{Measurement, Point3, Vector3};
use rand::prelude::*;
use std::f64::consts::PI;

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
) -> (Vec<Point3>, Vec<Measurement>) {
    let mut rng = thread_rng();
    let mut all_data = Vec::new();
    let mut true_targets = Vec::new();

    for i in 0..num_targets {
        // 直接在本地笛卡尔坐标系中生成目标位置
        let true_target_pos = Point3 {
            x: rng.gen_range(target_x_range.0..target_x_range.1),
            y: rng.gen_range(target_y_range.0..target_y_range.1),
            z: rng.gen_range(target_z_range.0..target_z_range.1),
        };
        true_targets.push(true_target_pos);

        println!("目标 {} 真实位置 (XYZ): X={:.2}, Y={:.2}, Z={:.2}",
            i + 1,
            true_target_pos.x,
            true_target_pos.y,
            true_target_pos.z
        );
        
        let num_stations = rng.gen_range(num_stations_per_target_range.0..=num_stations_per_target_range.1);
        for _ in 0..num_stations {
            // 直接在本地笛卡尔坐标系中生成站点位置
            let angle = rng.gen_range(0.0..2.0 * PI);
            let dist = rng.gen_range(station_dist_range.0..station_dist_range.1);
            let true_station_pos = Point3 {
                x: true_target_pos.x + dist * angle.cos(),
                y: true_target_pos.y + dist * angle.sin(),
                z: rng.gen_range(station_z_range.0..station_z_range.1),
            };

            let true_direction = (true_target_pos - true_station_pos).normalize();
            
            // 添加噪声
            let measured_station_pos = Point3 {
                x: true_station_pos.x + rng.gen_range(-pos_noise_std..pos_noise_std),
                y: true_station_pos.y + rng.gen_range(-pos_noise_std..pos_noise_std),
                z: true_station_pos.z + rng.gen_range(-alt_noise_std..alt_noise_std),
            };
            
            let measured_direction = Vector3::new(
                true_direction.x + rng.gen_range(-angle_noise_std..angle_noise_std),
                true_direction.y + rng.gen_range(-angle_noise_std..angle_noise_std),
                true_direction.z + rng.gen_range(-angle_noise_std..angle_noise_std),
            ).normalize();
            
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