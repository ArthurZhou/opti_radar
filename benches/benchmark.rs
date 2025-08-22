// benches/benchmark.rs
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use opti_radar::target_processor::find_targets;
use opti_radar::data_generator::generate_data;

/// 基准测试函数，用于测量 find_targets 的性能。
fn bench_find_targets(c: &mut Criterion) {
    // 数据生成只进行一次，避免在基准测试循环中重复执行
    let (_, all_data) = generate_data(
        10,          // num_targets
        (-500.0, 500.0),
        (-500.0, 500.0),
        (50.0, 150.0),
        (5, 10),     // num_stations_per_target_range
        (100.0, 500.0),
        (10.0, 30.0),
        1.0,         // pos_noise_std
        0.5,         // alt_noise_std
        0.005,       // angle_noise_std
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

// 定义基准测试组和主函数
criterion_group!(benches, bench_find_targets);
criterion_main!(benches);
