// src/data_generator.rs

use crate::target_processor::Measurement;
use nalgebra::{Point3, Vector3};
use rand::prelude::*;
use std::f64::consts::PI;

/// 生成模拟雷达测量数据和真实目标位置。
///
/// 此函数为多个目标创建一组测量数据，其中包括
/// 测量站位置和测量角度中逼真的噪声。
///
/// # 参数
/// * `num_targets` - 要生成的目标数量。
/// * `target_x_range`, `target_y_range`, `target_z_range` - 目标位置的最小/最大范围。
/// * `num_stations_per_target_range` - 每个目标对应的测量站数量的最小/最大范围元组。
/// * `station_dist_range` - 测量站与其各自目标之间距离的最小/最大范围。
/// * `station_z_range` - 测量站海拔高度的最小/最大范围。
/// * `pos_noise_std` - 测量站位置噪声的标准差。
/// * `alt_noise_std` - 测量站海拔高度噪声的标准差。
/// * `angle_noise_std` - 测量角度噪声的标准差。
///
/// # 返回值
/// 一个元组，包含：
/// * `Vec<Point3<f64>>` - 目标的真实、无噪声位置的向量。
/// * `Vec<Measurement>` - 生成的带噪声的测量数据的向量。
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
        // 直接在笛卡尔坐标系中生成目标位置
        let true_target_pos = Point3::new(
            rng.gen_range(target_x_range.0..target_x_range.1),
            rng.gen_range(target_y_range.0..target_y_range.1),
            rng.gen_range(target_z_range.0..target_z_range.1),
        );
        true_targets.push(true_target_pos);

        let num_stations =
            rng.gen_range(num_stations_per_target_range.0..=num_stations_per_target_range.1);
        for _ in 0..num_stations {
            // 直接在笛卡尔坐标系中生成测量站位置
            let angle = rng.gen_range(0.0..2.0 * PI);
            let dist = rng.gen_range(station_dist_range.0..station_dist_range.1);
            let true_station_pos = Point3::new(
                true_target_pos.x + dist * angle.cos(),
                true_target_pos.y + dist * angle.sin(),
                rng.gen_range(station_z_range.0..station_z_range.1),
            );

            let true_direction = (true_target_pos - true_station_pos).normalize();

            // 添加噪声
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