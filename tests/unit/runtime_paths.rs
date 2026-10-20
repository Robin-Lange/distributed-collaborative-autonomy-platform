use super::*;

fn unique_test_directory(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("dca-runtime-paths-{name}-{}", std::process::id()))
}

#[test]
fn source_run_uses_workspace_log_directory() {
    let root = unique_test_directory("source");
    let nested = root.join("target").join("release");
    std::fs::create_dir_all(root.join("apps").join("dca")).unwrap();
    std::fs::create_dir_all(root.join("platforms").join("spot")).unwrap();
    std::fs::create_dir_all(&nested).unwrap();
    std::fs::write(root.join("Cargo.toml"), "[workspace]\n").unwrap();

    let paths =
        RunPaths::for_executable(&nested.join("dca"), "20260727_120000".to_owned()).unwrap();

    assert_eq!(paths.log_directory(), root.join("logs"));
    assert_eq!(
        paths.operator_log_path(),
        root.join("logs").join("operator_ui_20260727_120000.md")
    );
    assert_eq!(
        paths.mcap_path(),
        root.join("logs")
            .join("mcap")
            .join("run_20260727_120000.mcap")
    );

    std::fs::remove_dir_all(root).unwrap();
}

#[test]
fn release_run_uses_bundle_directory() {
    let root = unique_test_directory("release");
    let bundle = root.join("release-bundle");
    std::fs::create_dir_all(&bundle).unwrap();

    let paths =
        RunPaths::for_executable(&bundle.join("dca"), "20260727_120001".to_owned()).unwrap();

    assert_eq!(paths.log_directory(), bundle.join("logs"));

    std::fs::remove_dir_all(root).unwrap();
}
