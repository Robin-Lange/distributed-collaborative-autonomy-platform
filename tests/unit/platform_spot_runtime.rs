use super::*;

#[test]
fn bundled_runtime_matrix_matches_the_release_matrix() {
    assert_eq!(
        RuntimeTarget::for_platform("linux", "x86_64")
            .expect("Ubuntu x86_64 is supported")
            .filename,
        "spot-sidecar-x86_64-unknown-linux-gnu"
    );
    assert_eq!(
        RuntimeTarget::for_platform("linux", "aarch64")
            .expect("Ubuntu ARM64 is supported")
            .filename,
        "spot-sidecar-aarch64-unknown-linux-gnu"
    );
    assert_eq!(
        RuntimeTarget::for_platform("macos", "aarch64")
            .expect("macOS ARM64 is supported")
            .filename,
        "spot-sidecar-aarch64-apple-darwin"
    );
    assert_eq!(
        RuntimeTarget::for_platform("windows", "x86_64")
            .expect("Windows x86_64 is supported")
            .filename,
        "spot-sidecar-x86_64-pc-windows-msvc.exe"
    );
    assert!(RuntimeTarget::for_platform("freebsd", "x86_64").is_err());
}

#[test]
fn bundled_runtime_is_relative_to_the_launcher() {
    let launcher = Path::new("release").join("dca");
    let path = bundled_runtime_path(&launcher, "spot-sidecar-x86_64-unknown-linux-gnu")
        .expect("launcher has a parent");

    assert_eq!(
        path,
        Path::new("release")
            .join("platforms")
            .join("spot")
            .join("spot-sidecar-x86_64-unknown-linux-gnu")
    );
}

#[test]
fn python_lookup_prefers_python_3_10() {
    let temporary = std::env::temp_dir().join(format!(
        "dca-platform-spot-runtime-test-{}",
        std::process::id()
    ));
    std::fs::create_dir_all(&temporary).expect("temporary directory");
    let python = temporary.join(python_candidates()[0].0);
    std::fs::write(&python, []).expect("fake interpreter");

    let interpreter =
        find_python_on_path(Some(temporary.as_os_str())).expect("interpreter is found");

    assert_eq!(interpreter.program, python);
    assert!(interpreter.arguments.is_empty());

    std::fs::remove_file(&python).expect("fake interpreter is removed");
    std::fs::remove_dir(&temporary).expect("temporary directory is removed");
}

#[test]
fn source_runtime_places_the_script_before_runtime_arguments() {
    let runtime = SpotRuntimeCommand::python(
        PythonInterpreter {
            program: PathBuf::from("python3.10"),
            arguments: vec![OsString::from("-I")],
        },
        PathBuf::from("platforms/spot/spot_sidecar.py"),
    );

    assert_eq!(runtime.program(), Path::new("python3.10"));
    assert_eq!(
        runtime.arguments(),
        [
            OsString::from("-I"),
            OsString::from("platforms/spot/spot_sidecar.py")
        ]
    );
}

#[test]
fn source_runtime_cache_uses_native_user_cache_conventions() {
    let home = OsStr::new("/users/operator");
    let local = OsStr::new("C:\\Users\\operator\\AppData\\Local");
    let xdg = OsStr::new("/var/cache/operator");

    assert_eq!(
        cache_base_directory("macos", None, None, Some(home)),
        Some(Path::new("/users/operator").join("Library").join("Caches"))
    );
    assert_eq!(
        cache_base_directory("windows", Some(local), None, Some(home)),
        Some(PathBuf::from(local))
    );
    assert_eq!(
        cache_base_directory("linux", None, Some(xdg), Some(home)),
        Some(PathBuf::from(xdg))
    );
    assert_eq!(
        cache_base_directory("linux", None, None, Some(home)),
        Some(Path::new("/users/operator").join(".cache"))
    );
}

#[test]
fn managed_python_path_matches_the_host_layout() {
    let environment = Path::new("cache").join("python-3.10");

    assert_eq!(
        venv_python_path(&environment, "macos"),
        environment.join("bin").join("python")
    );
    assert_eq!(
        venv_python_path(&environment, "windows"),
        environment.join("Scripts").join("python.exe")
    );
}
