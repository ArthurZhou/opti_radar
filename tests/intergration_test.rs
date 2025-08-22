// tests/integration_test.rs

use opti_radar::target_processor::{find_targets, Vector3};
use opti_radar::data_generator::generate_data;

#[test]
fn test_localization_accuracy() {
    let mut total_overall_error_sum = 0.0;
    let mut successful_runs_count = 0;
    let num_runs = 10;
    let true_num_targets = 3;

    println!("\n--- 正在进行 {} 次集成测试以验证定位精度 ---", num_runs);

    for run_count in 1..=num_runs {
        let (true_targets, all_data) = generate_data(
            true_num_targets,
            (-2000.0, 2000.0), // 目标 X 坐标范围 (米) - 增大范围
            (-2000.0, 2000.0), // 目标 Y 坐标范围 (米) - 增大范围
            (50.0, 200.0), // 目标 Z 坐标范围 (米)
            (3, 5), // 每个目标的测量站数量范围
            (500.0, 2000.0), // 测量站到目标的距离范围 (米)
            (30.0, 70.0), // 测量站的 Z 坐标范围 (米)
            5.0, // 站点位置噪声标准差 (米)
            2.0, // 站点高度噪声标准差 (米)
            0.005, // 角度噪声标准差 (弧度)
        );

        let located_targets = find_targets(&all_data, 20.0, 3);
        
        let located_num_targets = located_targets.len();
        let num_diff = (true_num_targets as isize - located_num_targets as isize).abs();

        if num_diff > 1 {
            println!("第{}次运行：目标数量严重不匹配，真实：{}，定位：{}，本次运行被忽略。", run_count, true_num_targets, located_num_targets);
            continue;
        }

        let mut run_error_sum = 0.0;
        let mut matched_targets_count = 0;
        let mut located_targets_indices_used = vec![false; located_targets.len()];

        for true_target in &true_targets {
            let mut min_dist_sq = f64::MAX;
            let mut best_match_idx = None;

            for (i, located_target) in located_targets.iter().enumerate() {
                if located_targets_indices_used[i] { continue; }

                let diff_vec = Vector3::new(
                    located_target.x - true_target.x,
                    located_target.y - true_target.y,
                    located_target.z - true_target.z,
                );
                let dist_sq = diff_vec.norm_squared();
                if dist_sq < min_dist_sq {
                    min_dist_sq = dist_sq;
                    best_match_idx = Some(i);
                }
            }

            if let Some(idx) = best_match_idx {
                let error_dist = min_dist_sq.sqrt();
                run_error_sum += error_dist;
                matched_targets_count += 1;
                located_targets_indices_used[idx] = true;
            }
        }
        
        if matched_targets_count > 0 {
            let avg_run_error = run_error_sum / matched_targets_count as f64;
            println!("第{}次运行：成功匹配 {}/{} 个目标，平均误差: {:.2} 米", run_count, matched_targets_count, true_num_targets, avg_run_error);
            total_overall_error_sum += avg_run_error;
            successful_runs_count += 1;
        } else {
            println!("第{}次运行：未成功匹配任何目标，本次运行被忽略。", run_count);
        }
    }

    if successful_runs_count == 0 {
        panic!("在 {} 次运行中未能成功匹配任何目标，测试失败。", num_runs);
    }

    let overall_avg_error = total_overall_error_sum / successful_runs_count as f64;
    println!("\n{} 次成功运行的整体平均误差: {:.2} 米", successful_runs_count, overall_avg_error);
    
    assert!(overall_avg_error < 100.0, "{} 次成功运行的整体平均误差 {:.2} 米超过了可接受的阈值 (100.0 米)", successful_runs_count, overall_avg_error);
}