// src/target_processor.rs

use nalgebra as na;
use na::{DMatrix, DVector, Matrix3, Point3, Vector3};
use rand::prelude::*;
use std::collections::HashSet;

// --- 数据结构 ---
// Measurement 表示原始传感器数据
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
    pub position: Point3<f64>, // 目标位置
    pub num_lines: usize,      // 用于拟合的光线数量
    pub avg_error_dist_m: f64, // 平均残差（米）
}

#[derive(Clone, Copy)]
pub struct Line {
    pub start: Point3<f64>,     // 光线起点
    pub direction: Vector3<f64>, // 单位化方向
}

/// Measurement → Line
fn get_line(m: &Measurement) -> Line {
    let start_point = Point3::new(m.x, m.y, m.z);
    let direction = Vector3::new(m.direction_x, m.direction_y, m.direction_z).normalize();
    Line {
        start: start_point,
        direction,
    }
}

/// 求两条光线之间的最近点中点
fn find_closest_midpoint(line1: &Line, line2: &Line) -> Point3<f64> {
    let w0 = line1.start - line2.start;
    let a = line1.direction.dot(&line1.direction);
    let b = line1.direction.dot(&line2.direction);
    let c = line2.direction.dot(&line2.direction);
    let d = line1.direction.dot(&w0);
    let e = line2.direction.dot(&w0);
    let denom = a * c - b * b;
    if denom.abs() < 1e-6 {
        // 平行或接近平行，直接返回起点平均
        return Point3::from((line1.start.coords + line2.start.coords) * 0.5);
    }
    let s = (b * e - c * d) / denom;
    let t = (a * e - b * d) / denom;
    let closest_point1 = line1.start + line1.direction * s;
    let closest_point2 = line2.start + line2.direction * t;
    Point3::from((closest_point1.coords + closest_point2.coords) * 0.5)
}

/// 使用 Levenberg-Marquardt 优化点到多条光线的残差
///
/// 残差定义为：点到每条光线的垂直向量 `distance_vec`
/// 维度为 `3n`，LM 会最小化所有残差向量的平方和。
pub fn levenberg_marquardt_optimize(
    lines: &[Line],
    initial_guess: Point3<f64>,
    iterations: usize,
    initial_lambda: f64,
) -> Point3<f64> {
    let mut current_pos = initial_guess;
    let mut lambda = initial_lambda;
    let lambda_factor_up = 10.0;
    let lambda_factor_down = 0.1;

    for _ in 0..iterations {
        let n = lines.len();
        let mut j = DMatrix::zeros(3 * n, 3);
        let mut e = DVector::zeros(3 * n);

        // 构建残差向量 e 和雅可比矩阵 J
        for (i, line) in lines.iter().enumerate() {
            let pa = current_pos - line.start;
            let proj = pa.dot(&line.direction);
            let distance_vec = pa - line.direction * proj; // 垂直分量

            // 残差
            e.rows_mut(3 * i, 3).copy_from(&DVector::from_column_slice(distance_vec.as_slice()));

            // 雅可比：残差 = (p - start) - d ( (p - start)·d )
            // 对 p 的导数 ≈ I - d dᵀ
            let jac_block = Matrix3::identity() - line.direction * line.direction.transpose();
            j
                .view_mut((3 * i, 0), (3, 3))
                .copy_from(&jac_block);
        }

        let j_t = j.transpose();
        let h_approx = &j_t * &j;
        let b = &j_t * &e;

        // LM 更新： (H + λI) Δp = -b
        let h_lm = h_approx + Matrix3::identity() * lambda;
        let delta = match h_lm.try_inverse() {
            Some(inv_h) => inv_h * -b,
            None => {
                lambda *= lambda_factor_up;
                continue;
            }
        };

        let delta_vec = Vector3::new(delta[0], delta[1], delta[2]);
        let new_pos = current_pos + delta_vec;

        // 计算误差平方和
        let mut new_error_sq = 0.0;
        for line in lines.iter() {
            let pa = new_pos - line.start;
            let proj = pa.dot(&line.direction);
            let dist_vec = pa - line.direction * proj;
            new_error_sq += dist_vec.norm_squared();
        }
        let current_error_sq: f64 = e.norm_squared();

        // 接受或拒绝更新
        if new_error_sq < current_error_sq {
            current_pos = new_pos;
            lambda *= lambda_factor_down; // 更接近高斯牛顿
        } else {
            lambda *= lambda_factor_up; // 更接近梯度下降
        }
    }
    current_pos
}

/// RANSAC 拟合光线集合，寻找最大内点集
pub fn ransac_fit_lines(
    all_lines: &[Line],
    ransac_iterations: usize,
    ransac_threshold: f64,
    min_lines: usize,
) -> Option<(Point3<f64>, Vec<usize>)> {
    let mut rng = thread_rng();
    let mut best_inliers_indices = Vec::new();
    let mut best_model_pos = Point3::new(0.0, 0.0, 0.0);

    if all_lines.len() < 3 {
        return None;
    }

    for _ in 0..ransac_iterations {
        // 随机选取 3 条线
        let mut sample_indices = HashSet::new();
        while sample_indices.len() < 3 {
            sample_indices.insert(rng.gen_range(0..all_lines.len()));
        }
        let sample_indices_vec: Vec<_> = sample_indices.iter().copied().collect();
        let sample_lines: Vec<_> = sample_indices_vec.iter().map(|&i| all_lines[i]).collect();

        // 初始猜测：3 条光线两两最近点的平均
        let initial_guess = (find_closest_midpoint(&sample_lines[0], &sample_lines[1]).coords
            + find_closest_midpoint(&sample_lines[0], &sample_lines[2]).coords
            + find_closest_midpoint(&sample_lines[1], &sample_lines[2]).coords)
            / 3.0;
        let initial_guess = Point3::from(initial_guess);

        // 统计内点
        let mut current_inliers_indices = Vec::new();
        for (i, line) in all_lines.iter().enumerate() {
            let pa = initial_guess - line.start;
            let proj = pa.dot(&line.direction);
            let distance = (pa - line.direction * proj).norm();
            if distance < ransac_threshold {
                current_inliers_indices.push(i);
            }
        }

        if current_inliers_indices.len() > best_inliers_indices.len()
            && current_inliers_indices.len() >= min_lines
        {
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

/// 综合使用 RANSAC + LM 定位多个目标
pub fn find_targets(
    data: &[Measurement],
    ransac_threshold_m: f64,
    min_lines_per_target: usize,
) -> Vec<LocatedTarget> {
    let all_lines: Vec<_> = data.iter().map(get_line).collect();
    let mut located_targets = Vec::new();
    let mut used_line_indices = HashSet::new();
    let mut target_id_counter = 1;

    if all_lines.len() < min_lines_per_target {
        return located_targets;
    }

    loop {
        // 筛选未使用的光线
        let remaining_lines_map: Vec<_> = all_lines
            .iter()
            .enumerate()
            .filter(|(i, _)| !used_line_indices.contains(i))
            .collect();
        let remaining_lines: Vec<_> = remaining_lines_map.iter().map(|(_, l)| **l).collect();

        if remaining_lines.len() < min_lines_per_target {
            break;
        }

        if let Some((initial_guess, inliers_indices)) = ransac_fit_lines(
            &remaining_lines,
            100,
            ransac_threshold_m,
            min_lines_per_target,
        ) {
            let actual_inliers_indices: Vec<_> = inliers_indices
                .iter()
                .map(|&i| remaining_lines_map[i].0)
                .collect();
            let target_lines: Vec<_> = actual_inliers_indices
                .iter()
                .map(|&i| all_lines[i])
                .collect();

            // LM 优化
            let final_pos = levenberg_marquardt_optimize(&target_lines, initial_guess, 200, 0.001);

            // 计算平均残差
            let mut total_error_sq = 0.0;
            for line in &target_lines {
                let pa = final_pos - line.start;
                let proj = pa.dot(&line.direction);
                let dist_vec = pa - line.direction * proj;
                total_error_sq += dist_vec.norm_squared();
            }
            let avg_error_dist = (total_error_sq / target_lines.len() as f64).sqrt();

            located_targets.push(LocatedTarget {
                id: format!("Target_{}", target_id_counter),
                position: final_pos,
                num_lines: target_lines.len(),
                avg_error_dist_m: avg_error_dist,
            });
            target_id_counter += 1;

            for &i in &actual_inliers_indices {
                used_line_indices.insert(i);
            }
        } else {
            break;
        }
    }

    located_targets
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_line_normalization() {
        let measurement = Measurement {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            direction_x: 3.0,
            direction_y: 4.0,
            direction_z: 0.0,
        };
        let line = get_line(&measurement);
        assert!((line.direction.norm() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_find_closest_midpoint() {
        let line1 = Line {
            start: Point3::new(0.0, 5.0, 0.0),
            direction: Vector3::new(1.0, 0.0, 0.0).normalize(),
        };
        let line2 = Line {
            start: Point3::new(5.0, 0.0, 0.0),
            direction: Vector3::new(0.0, 1.0, 0.0).normalize(),
        };
        let midpoint = find_closest_midpoint(&line1, &line2);
        let epsilon = 1e-6;
        assert!((midpoint.x - 5.0).abs() < epsilon);
        assert!((midpoint.y - 5.0).abs() < epsilon);
        assert!((midpoint.z - 0.0).abs() < epsilon);
    }

    #[test]
    fn test_ransac_fit_lines() {
        let mut lines = Vec::new();
        for _ in 0..10 {
            let start = Point3::new(
                thread_rng().gen_range(0.0..5.0),
                thread_rng().gen_range(15.0..25.0),
                thread_rng().gen_range(25.0..35.0),
            );
            let direction = (Point3::new(10.0, 20.0, 30.0) - start).normalize();
            lines.push(Line { start, direction });
        }
        for _ in 0..5 {
            let start = Point3::new(
                thread_rng().gen_range(-50.0..50.0),
                thread_rng().gen_range(-50.0..50.0),
                thread_rng().gen_range(-50.0..50.0),
            );
            let direction =
                Vector3::new(thread_rng().gen(), thread_rng().gen(), thread_rng().gen()).normalize();
            lines.push(Line { start, direction });
        }

        let result = ransac_fit_lines(&lines, 100, 1.0, 3);
        assert!(result.is_some());

        if let Some((initial_guess, inliers_indices)) = result {
            assert!(inliers_indices.len() >= 8);
            let epsilon = 1e-1;
            assert!((initial_guess.x - 10.0).abs() < epsilon);
            assert!((initial_guess.y - 20.0).abs() < epsilon);
            assert!((initial_guess.z - 30.0).abs() < epsilon);
        }
    }

    #[test]
    fn test_levenberg_marquardt_with_perfect_data() {
        let line1 = Line {
            start: Point3::new(-10.0, 0.0, 10.0),
            direction: Vector3::new(1.0, 0.0, 0.0),
        };
        let line2 = Line {
            start: Point3::new(0.0, -10.0, 10.0),
            direction: Vector3::new(0.0, 1.0, 0.0),
        };
        let lines = vec![line1, line2];

        let initial_guess = Point3::new(100.0, 100.0, 100.0);
        let initial_lambda = 0.01;
        let iterations = 200;

        let final_pos =
            levenberg_marquardt_optimize(&lines, initial_guess, iterations, initial_lambda);
        let epsilon = 1e-6;

        assert!((final_pos.x - 0.0).abs() < epsilon);
        assert!((final_pos.y - 0.0).abs() < epsilon);
        assert!((final_pos.z - 10.0).abs() < epsilon);
    }
}
