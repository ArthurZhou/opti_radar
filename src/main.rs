fn main() {
    const VER: &str = env!("CARGO_PKG_VERSION");
    println!();
    println!("opti_radar central processor ver{}", VER);
    println!("Run 'cargo test' to execute the tests.");
    println!("Run 'cargo bench' to execute the benchmarks.");
    println!();
}
