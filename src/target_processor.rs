// src/target_processor.rs

// 导入 nalgebra 库
use nalgebra as na;
use na::{Point3, Vector3};
use rand::prelude::*;
use std::collections::HashSet;

// --- 数据结构 ---
// Measurement 代表原始的传感器数据，保留 f64 类型
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
    pub position: Point3<f64>, // 使用 Point3 存储位置
    pub num_lines: usize,
    pub avg_error_dist_m: f64,
}

#[derive(Clone, Copy)]
pub struct Line {
    pub start: Point3<f64>,
    pub direction: Vector3<f64>,
}

/// 将 Measurement 结构体转换为 Line 结构体，并归一化方向向量。
///
/// # 参数
/// * `m` - 一个 `Measurement` 的引用。
///
/// # 返回值
/// 相应的 `Line` 结构体，其中方向向量已被归一化。
fn get_line(m: &Measurement) -> Line {
    let start_point = Point3::new(m.x, m.y, m.z);
    let direction = Vector3::new(m.direction_x, m.direction_y, m.direction_z).normalize();
    Line {
        start: start_point,
        direction,
    }
}

/// 计算两条空间光线之间最短距离的中点。
///
/// 此中点作为这两条光线所代表的目标位置的一个初步近似。
///
/// # 参数
/// * `line1` - 第一个 `Line` 的引用。
/// * `line2` - 第二个 `Line` 的引用。
///
/// # 返回值
/// 一个 `Point3<f64>`，表示两光线最短距离的中点。
fn find_closest_midpoint(line1: &Line, line2: &Line) -> Point3<f64> {
    let w0 = line1.start - line2.start;
    let a = line1.direction.dot(&line1.direction);
    let b = line1.direction.dot(&line2.direction);
    let c = line2.direction.dot(&line2.direction);
    let d = line1.direction.dot(&w0);
    let e = line2.direction.dot(&w0);
    let denom = a * c - b * b;
    if denom.abs() < 1e-6 {
        return Point3::from((line1.start.coords + line2.start.coords) * 0.5);
    }
    let s = (b * e - c * d) / denom;
    let t = (a * e - b * d) / denom;
    let closest_point1 = line1.start + line1.direction * s;
    let closest_point2 = line2.start + line2.direction * t;
    Point3::from((closest_point1.coords + closest_point2.coords) * 0.5)
}

/// 使用梯度下降法优化目标位置。
///
/// 它通过最小化目标点到所有光线的距离平方和来寻找最优解。
///
/// # 参数
/// * `lines` - 参与优化的光线集合。
/// * `initial_guess` - 优化过程的初始猜测位置。
/// * `learning_rate` - 梯度下降的学习率，控制每次迭代的步长。
/// * `iterations` - 梯度下降的迭代次数。
///
/// # 返回值
/// 一个 `Point3<f64>`，表示经过优化后的目标位置。
pub fn gradient_descent_optimize(
    lines: &[Line],
    initial_guess: Point3<f64>,
    learning_rate: f64,
    iterations: usize,
) -> Point3<f64> {
    let mut current_pos = initial_guess;
    for _ in 0..iterations {
        let mut total_gradient = Vector3::zeros();
        for line in lines {
            let pa = current_pos - line.start;
            let a = line.direction;
            let projection_factor = pa.dot(&a);
            let distance_vec = pa - a * projection_factor;
            let gradient = distance_vec * 2.0;
            total_gradient += gradient;
        }
        current_pos -= total_gradient * learning_rate;
    }
    current_pos
}

/// 使用 RANSAC 算法从包含异常值的光线中，找到最佳拟合的光线子集（即“内点”）。
///
/// # 参数
/// * `all_lines` - 包含所有测量光线的向量。
/// * `ransac_iterations` - RANSAC 算法的迭代次数。
/// * `ransac_threshold` - 判断一条光线是否为内点的距离阈值。
/// * `min_lines` - 识别一个目标所需的最小内点数量，低于此数量则认为未找到有效模型。
///
/// # 返回值
/// 一个 `Option<(Point3<f64>, Vec<usize>)>`。如果找到一个有效的模型，它将返回一个包含初始猜测位置和内点索引的元组；否则返回 `None`。
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
        let mut sample_indices = HashSet::new();
        while sample_indices.len() < 3 {
            sample_indices.insert(rng.gen_range(0..all_lines.len()));
        }
        let sample_indices_vec: Vec<_> = sample_indices.iter().copied().collect();
        let sample_lines: Vec<_> = sample_indices_vec.iter().map(|&i| all_lines[i]).collect();

        let initial_guess = (find_closest_midpoint(&sample_lines[0], &sample_lines[1]).coords
            + find_closest_midpoint(&sample_lines[0], &sample_lines[2]).coords
            + find_closest_midpoint(&sample_lines[1], &sample_lines[2]).coords)
            / 3.0;
        let initial_guess = Point3::from(initial_guess);

        let mut current_inliers_indices = Vec::new();
        for (i, line) in all_lines.iter().enumerate() {
            let pa = initial_guess - line.start;
            let projection_factor = pa.dot(&line.direction);
            let distance = (pa - line.direction * projection_factor).norm();
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

/// 从所有测量数据中识别并定位多个目标。
///
/// 它通过一个循环，重复执行 RANSAC 和梯度下降过程，直到所有可识别的目标都被找到。
///
/// # 参数
/// * `data` - 包含所有原始 `Measurement` 数据的切片。
/// * `ransac_threshold_m` - RANSAC 算法的距离阈值。
/// * `min_lines_per_target` - 识别一个目标所需的最小测量线数量。
///
/// # 返回值
/// 一个 `Vec<LocatedTarget>`，其中包含所有成功定位的目标信息。
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
    if num_measurements < min_lines_per_target {
        return located_targets;
    }

    loop {
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

    // 测试 get_line 函数，确保方向向量被正确归一化
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

    // 测试 find_closest_midpoint 函数
    #[test]
    fn test_find_closest_midpoint() {
        // 两条光线在 (5, 5, 0) 处相交
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
    
    // 测试 ransac_fit_lines 函数
    #[test]
    fn test_ransac_fit_lines() {
        let mut lines = Vec::new();
        // 添加 10 条完美的内点光线，相交于 (10, 20, 30)
        for _ in 0..10 {
            let start = Point3::new(thread_rng().gen_range(0.0..5.0), thread_rng().gen_range(15.0..25.0), thread_rng().gen_range(25.0..35.0));
            let direction = (Point3::new(10.0, 20.0, 30.0) - start).normalize();
            lines.push(Line { start, direction });
        }
        // 添加 5 条噪声光线（异常值）
        for _ in 0..5 {
            let start = Point3::new(thread_rng().gen_range(-50.0..50.0), thread_rng().gen_range(-50.0..50.0), thread_rng().gen_range(-50.0..50.0));
            let direction = Vector3::new(thread_rng().gen(), thread_rng().gen(), thread_rng().gen()).normalize();
            lines.push(Line { start, direction });
        }

        let result = ransac_fit_lines(&lines, 100, 1.0, 3);
        assert!(result.is_some());
        
        if let Some((initial_guess, inliers_indices)) = result {
            // 确保至少找到了 8 条内点
            assert!(inliers_indices.len() >= 8);
            // 确保 RANSAC 找到的初始猜测位置接近真实位置
            let epsilon = 1e-1;
            assert!((initial_guess.x - 10.0).abs() < epsilon);
            assert!((initial_guess.y - 20.0).abs() < epsilon);
            assert!((initial_guess.z - 30.0).abs() < epsilon);
        }
    }

    // 梯度下降测试
    #[test]
    fn test_gradient_descent_with_perfect_data() {
        // 创建一个简单的场景：两条光线完美相交于 (0, 0, 10)
        let line1 = Line {
            start: Point3::new(-10.0, 0.0, 10.0),
            direction: Vector3::new(1.0, 0.0, 0.0),
        };
        let line2 = Line {
            start: Point3::new(0.0, -10.0, 10.0),
            direction: Vector3::new(0.0, 1.0, 0.0),
        };
        let lines = vec![line1, line2];

        // 设定一个初始猜测，它离真实解不远
        let initial_guess = Point3::new(1.0, 1.0, 10.0);
        let learning_rate = 0.01;
        // 增加迭代次数以保证收敛
        let iterations = 1000;

        // 运行梯度下降优化
        let final_pos = gradient_descent_optimize(&lines, initial_guess, learning_rate, iterations);
        let epsilon = 1e-4;

        // 断言最终位置非常接近 (0, 0, 10)
        assert!((final_pos.x - 0.0).abs() < epsilon);
        assert!((final_pos.y - 0.0).abs() < epsilon);
        assert!((final_pos.z - 10.0).abs() < epsilon);
    }
}
