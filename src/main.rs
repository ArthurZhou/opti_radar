// src/main.rs
use opti_radar::{target_processor::{find_targets, LocatedTarget}, data_generator::generate_data};

fn main() {
    // 数据生成参数
    let (true_targets, measurements) = generate_data(
        5,                     // num_targets
        (0.0, 50.0),           // target_x_range
        (0.0, 50.0),           // target_y_range
        (0.0, 20.0),           // target_z_range
        (3, 6),                // num_stations_per_target_range
        (5.0, 15.0),           // station_dist_range
        (1.0, 5.0),            // station_z_range
        0.1,                   // pos_noise_std
        0.2,                   // alt_noise_std
        0.01,                  // angle_noise_std
    );

    let located_targets: Vec<LocatedTarget> = find_targets(&measurements, 1.0, 3);

    // 输出 CSV：TargetID, TrueX, TrueY, TrueZ, EstX, EstY, EstZ, AvgError
    println!("TargetID,TrueX,TrueY,TrueZ,EstX,EstY,EstZ,AvgError");
    for (i, est) in located_targets.iter().enumerate() {
        let true_pos = &true_targets[i];
        println!("{},{},{},{},{},{},{},{}",
            est.id,
            true_pos.x,
            true_pos.y,
            true_pos.z,
            est.position.x,
            est.position.y,
            est.position.z,
            est.avg_error_dist_m
        );
    }
}
