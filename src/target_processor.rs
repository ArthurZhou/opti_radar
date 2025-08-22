// src/target_processor.rs

use std::collections::HashSet;
use std::ops::{Add, Sub, Mul};
use rand::prelude::*;

// --- 纯 Rust 实现：自定义3D向量和点 ---
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Vector3 { x, y, z }
    }
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
    pub fn norm_squared(&self) -> f64 {
        self.dot(self)
    }
    pub fn norm(&self) -> f64 {
        self.norm_squared().sqrt()
    }
    pub fn normalize(&self) -> Self {
        let n = self.norm();
        if n > 1e-10 {
            Vector3::new(self.x / n, self.y / n, self.z / n)
        } else {
            Vector3::new(0.0, 0.0, 0.0)
        }
    }
}
impl Add for Point3 {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Point3 { x: self.x + other.x, y: self.y + other.y, z: self.z + other.z }
    }
}
impl Sub for Point3 {
    type Output = Vector3;
    fn sub(self, other: Self) -> Vector3 {
        Vector3::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}
impl Add<Vector3> for Point3 {
    type Output = Self;
    fn add(self, other: Vector3) -> Self {
        Point3 { x: self.x + other.x, y: self.y + other.y, z: self.z + other.z }
    }
}
impl Add for Vector3 {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Vector3::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}
impl Sub for Vector3 {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Vector3::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}
impl Mul<f64> for Vector3 {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Vector3::new(self.x * scalar, self.y * scalar, self.z * scalar)
    }
}
impl Mul<f64> for Point3 {
    type Output = Point3;
    fn mul(self, scalar: f64) -> Point3 {
        Point3 { x: self.x * scalar, y: self.y * scalar, z: self.z * scalar }
    }
}

// --- 数据结构 ---
pub struct Measurement {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub direction_x: f64,
    pub direction_y: f64,
    pub direction_z: f64,
}

#[derive(Debug, Clone)]
pub struct LocatedTarget {
    pub id: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub num_lines: usize,
    pub avg_error_dist_m: f64,
}

#[derive(Clone, Copy)]
struct Line {
    start: Point3,
    direction: Vector3,
}

fn get_line(m: &Measurement) -> Line {
    let start_point = Point3 { x: m.x, y: m.y, z: m.z };
    let direction = Vector3::new(m.direction_x, m.direction_y, m.direction_z).normalize();
    Line { start: start_point, direction }
}

fn find_closest_midpoint(line1: &Line, line2: &Line) -> Point3 {
    let w0 = line1.start - line2.start;
    let a = line1.direction.dot(&line1.direction);
    let b = line1.direction.dot(&line2.direction);
    let c = line2.direction.dot(&line2.direction);
    let d = line1.direction.dot(&w0);
    let e = line2.direction.dot(&w0);
    let denom = a * c - b * b;
    if denom.abs() < 1e-6 {
        return Point3 { x: (line1.start.x + line2.start.x) * 0.5, y: (line1.start.y + line2.start.y) * 0.5, z: (line1.start.z + line2.start.z) * 0.5 };
    }
    let s = (b * e - c * d) / denom;
    let t = (a * e - b * d) / denom;
    let closest_point1 = line1.start + line1.direction * s;
    let closest_point2 = line2.start + line2.direction * t;
    Point3 { x: (closest_point1.x + closest_point2.x) * 0.5, y: (closest_point1.y + closest_point2.y) * 0.5, z: (closest_point1.z + closest_point2.z) * 0.5 }
}

fn gradient_descent_optimize(
    lines: &[Line],
    initial_guess: Point3,
    learning_rate: f64,
    iterations: usize,
) -> Point3 {
    let mut current_pos = initial_guess;
    for _ in 0..iterations {
        let mut total_gradient = Vector3::new(0.0, 0.0, 0.0);
        for line in lines {
            let pa = current_pos - line.start;
            let a = line.direction;
            let projection_factor = pa.dot(&a);
            let distance_vec = pa - a * projection_factor;
            let gradient = distance_vec * 2.0;
            total_gradient = total_gradient + gradient;
        }
        current_pos = current_pos + total_gradient * -learning_rate;
    }
    current_pos
}

// 修正后的 RANSAC 算法，确保它返回的内点索引与传入的光线列表相对应
fn ransac_fit_lines(all_lines: &[Line], ransac_iterations: usize, ransac_threshold: f64, min_lines: usize) -> Option<(Point3, Vec<usize>)> {
    let mut rng = thread_rng();
    let mut best_inliers_indices = Vec::new();
    let mut best_model_pos = Point3 { x: 0.0, y: 0.0, z: 0.0 };

    if all_lines.len() < 3 { return None; }

    for _ in 0..ransac_iterations {
        // 随机选择 3 条光线
        let mut sample_indices = HashSet::new();
        while sample_indices.len() < 3 {
            sample_indices.insert(rng.gen_range(0..all_lines.len()));
        }
        let sample_indices_vec: Vec<_> = sample_indices.iter().copied().collect();
        let sample_lines: Vec<_> = sample_indices_vec.iter().map(|&i| all_lines[i]).collect();

        // 计算这三条光线的交点作为初始模型
        let initial_guess = (find_closest_midpoint(&sample_lines[0], &sample_lines[1]) + find_closest_midpoint(&sample_lines[0], &sample_lines[2]) + find_closest_midpoint(&sample_lines[1], &sample_lines[2])) * (1.0 / 3.0);
        
        let mut current_inliers_indices = Vec::new();
        for (i, line) in all_lines.iter().enumerate() {
            let pa = initial_guess - line.start;
            let projection_factor = pa.dot(&line.direction);
            let distance = (pa - line.direction * projection_factor).norm();
            if distance < ransac_threshold {
                current_inliers_indices.push(i);
            }
        }

        if current_inliers_indices.len() > best_inliers_indices.len() && current_inliers_indices.len() >= min_lines {
            best_inliers_indices = current_inliers_indices;
            best_model_pos = initial_guess;
        }
    }
    
    if best_inliers_indices.len() >= min_lines {
        Some((best_model_pos, best_inliers_indices))
    } else {
        None
    }
}

pub fn find_targets(
    data: &[Measurement],
    ransac_threshold_m: f64,
    min_lines_per_target: usize,
) -> Vec<LocatedTarget> {
    let all_lines: Vec<_> = data.iter().map(get_line).collect();
    let mut located_targets = Vec::new();
    let mut used_line_indices = HashSet::new();
    let mut target_id_counter = 1;

    let num_measurements = all_lines.len();
    if num_measurements < min_lines_per_target { return located_targets; }

    loop {
        // 筛选出所有未被使用的光线
        let remaining_lines_map: Vec<_> = all_lines.iter().enumerate().filter(|(i, _)| !used_line_indices.contains(i)).collect();
        let remaining_lines: Vec<_> = remaining_lines_map.iter().map(|(_, l)| **l).collect();

        if remaining_lines.len() < min_lines_per_target {
            break;
        }

        // 运行 RANSAC 算法寻找下一个目标
        if let Some((initial_guess, inliers_indices)) = ransac_fit_lines(
            &remaining_lines,
            100,
            ransac_threshold_m,
            min_lines_per_target
        ) {
            // 获取实际的光线索引
            let actual_inliers_indices: Vec<_> = inliers_indices.iter().map(|&i| remaining_lines_map[i].0).collect();
            let target_lines: Vec<_> = actual_inliers_indices.iter().map(|&i| all_lines[i]).collect();
            
            // 使用内点进行梯度下降优化
            let final_pos = gradient_descent_optimize(&target_lines, initial_guess, 0.001, 200);

            let mut total_error_sq = 0.0;
            for line in &target_lines {
                let pa = final_pos - line.start;
                let projection_factor = pa.dot(&line.direction);
                let distance_vec = pa - line.direction * projection_factor;
                total_error_sq += distance_vec.norm_squared();
            }
            let avg_error_dist = (total_error_sq / target_lines.len() as f64).sqrt();

            located_targets.push(LocatedTarget {
                id: format!("Target_{}", target_id_counter),
                x: final_pos.x,
                y: final_pos.y,
                z: final_pos.z,
                num_lines: target_lines.len(),
                avg_error_dist_m: avg_error_dist,
            });
            target_id_counter += 1;

            // 标记已使用的光线，确保它们不会被再次使用
            for &i in &actual_inliers_indices {
                used_line_indices.insert(i);
            }
        } else {
            // RANSAC 没有找到足够的内点，停止搜索
            break;
        }
    }

    located_targets
}