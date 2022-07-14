use std::env;
use std::process::Command;

fn main() {
    let base_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    Command::new("cargo")
        .arg("fmt")
        .current_dir(&base_dir)
        .output()
        .unwrap();
    Command::new("cargo")
        .arg("--generate-lockfile")
        .current_dir(&base_dir)
        .output()
        .unwrap();
}
