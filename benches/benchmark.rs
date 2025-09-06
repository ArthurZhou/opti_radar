// benches/benchmark.rs
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use opti_radar::target_processor::{find_targets, ransac_fit_lines, levenberg_marquardt_optimize, Line};
use opti_radar::data_generator::generate_data;
use nalgebra::{Point3, Vector3};
use rand::{thread_rng, Rng};

/// 基准测试函数，用于测量 find_targets 的性能。
fn bench_find_targets(c: &mut Criterion) {
    // 数据生成只进行一次，避免在基准测试循环中重复执行
    let (_, all_data) = generate_data(
        10,            // num_targets
        (-500.0, 500.0),
        (-500.0, 500.0),
        (50.0, 150.0),
        (5, 10),       // num_stations_per_target_range
        (100.0, 500.0),
        (10.0, 30.0),
        1.0,           // pos_noise_std
        0.5,           // alt_noise_std
        0.005,         // angle_noise_std
    );

    let threshold = 20.0;
    let min_lines = 3;

    // 测量 find_targets 函数的性能
    c.bench_function("find_targets_10_targets", |b| {
        b.iter(|| {
            // 使用 black_box 防止编译器优化掉对结果的使用
            let located = find_targets(black_box(&all_data), black_box(threshold), black_box(min_lines));
            black_box(located);
        });
    });
}

/// 基准测试函数，用于测量 ransac_fit_lines 的性能。
fn bench_ransac(c: &mut Criterion) {
    // 准备测试数据
    let mut lines = Vec::new();
    for _ in 0..10 {
        let start = Point3::new(0.0, 0.0, 0.0);
        let direction = Vector3::new(1.0, 0.0, 0.0).normalize();
        lines.push(Line { start, direction });
    }
    for _ in 0..5 {
        let start = Point3::new(50.0, 50.0, 50.0);
        let direction = Vector3::new(0.0, 1.0, 0.0).normalize();
        lines.push(Line { start, direction });
    }
    let ransac_iterations = 100;
    let ransac_threshold = 1.0;
    let min_lines = 3;

    c.bench_function("ransac_fit_lines", |b| {
        b.iter(|| {
            let result = ransac_fit_lines(
                black_box(&lines),
                black_box(ransac_iterations),
                black_box(ransac_threshold),
                black_box(min_lines),
            );
            black_box(result);
        });
    });
}

/// 基准测试函数，用于测量 levenberg_marquardt_optimize 的性能。
fn bench_lm(c: &mut Criterion) {
    // 准备一组基准数据，模拟RANSAC筛选出的内点
    let mut lines = Vec::new();
    let true_position = Point3::new(10.0, 20.0, 30.0);
    let mut rng = thread_rng();

    // 生成10条完美拟合的光线，带一点随机噪声
    for _ in 0..10 {
        let start = Point3::new(
            rng.gen_range(0.0..5.0),
            rng.gen_range(15.0..25.0),
            rng.gen_range(25.0..35.0),
        );
        let direction = (true_position - start).normalize();
        lines.push(Line { start, direction });
    }

    let initial_guess = Point3::new(9.0, 19.0, 29.0);
    let iterations = 200;
    let initial_lambda = 0.001;

    c.bench_function("levenberg_marquardt_optimize", |b| {
        b.iter(|| {
            let result = levenberg_marquardt_optimize(
                black_box(&lines),
                black_box(initial_guess),
                black_box(iterations),
                black_box(initial_lambda),
            );
            black_box(result);
        });
    });
}

// 定义基准测试组和主函数
criterion_group!(benches, bench_find_targets, bench_ransac, bench_lm);
criterion_main!(benches);