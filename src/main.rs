// src/main.rs

use opti_radar::target_processor::find_targets;
use opti_radar::data_generator::generate_data;

fn main() {
    println!("--- 正在生成模拟数据 ---");
    let (_, all_data) = generate_data(
        3, // 目标数量
        (-500.0, 500.0), // 目标 X 坐标范围 (米)
        (-500.0, 500.0), // 目标 Y 坐标范围 (米)
        (50.0, 200.0), // 目标 Z 坐标范围 (米)
        (3, 5), // 每个目标的测量站数量范围
        (500.0, 2000.0), // 测量站到目标的距离范围 (米)
        (30.0, 70.0), // 测量站的 Z 坐标范围 (米)
        5.0, // 站点位置噪声标准差 (米)
        2.0, // 站点高度噪声标准差 (米)
        0.005, // 角度噪声标准差 (弧度)
    );

    println!("\n生成 {} 条观测数据。", all_data.len());
    println!("--- 正在调用处理器进行自动定位 ---");

    let located_targets = find_targets(&all_data, 50.0, 3);

    println!("\n--- 自动定位结果 ---");
    if located_targets.is_empty() {
        println!("没有找到足够的数据来识别任何目标。");
    } else {
        for target in located_targets {
            println!("目标ID: {}", target.id);
            println!("X: {:.2}", target.x);
            println!("Y: {:.2}", target.y);
            println!("Z: {:.2} 米", target.z);
            println!("参与测量的站点数量: {}", target.num_lines);
            println!("平均误差距离: {:.2} 米", target.avg_error_dist_m);
            println!("------------------------------");
        }
    }
}