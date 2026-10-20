//! Selection of the Spot sidecar shipped with a release or source checkout.

use std::{
    env,
    ffi::{OsStr, OsString},
    fs, io,
    path::{Path, PathBuf},
    process::{Command, Output},
};

const RUNTIME_OVERRIDE_ENV: &str = "DCA_SPOT_RUNTIME";
const SOURCE_RUNTIME_REQUIREMENTS: &str = include_str!("../requirements-runtime.txt");
const SOURCE_RUNTIME_IMPORT_CHECK: &str = "import bosdyn.client, google.protobuf, numpy, zenoh";

/// Program and leading arguments used to start the Spot sidecar.
///
/// Release bundles execute a native sidecar directly. Source checkouts execute
/// `spot_sidecar.py` with an available Python interpreter.
pub struct SpotRuntimeCommand {
    program: PathBuf,
    arguments: Vec<OsString>,
}

impl SpotRuntimeCommand {
    const fn executable(program: PathBuf) -> Self {
        Self {
            program,
            arguments: Vec::new(),
        }
    }

    fn python(interpreter: PythonInterpreter, script: PathBuf) -> Self {
        let mut arguments = interpreter.arguments;
        arguments.push(script.into_os_string());
        Self {
            program: interpreter.program,
            arguments,
        }
    }

    /// Returns the executable passed to `std::process::Command`.
    #[must_use]
    pub fn program(&self) -> &Path {
        &self.program
    }

    /// Returns arguments that must precede the sidecar's runtime arguments.
    #[must_use]
    pub fn arguments(&self) -> &[OsString] {
        &self.arguments
    }
}

/// Locates the standalone Spot sidecar for the current process.
///
/// The explicit `DCA_SPOT_RUNTIME` override takes precedence. A release build
/// then uses the native sidecar under `platforms/spot` beside the `dca`
/// executable. When running from a source checkout, the checked-in Python
/// sidecar is used instead.
///
/// # Errors
///
/// Returns an error when an override is invalid or neither a bundled runtime
/// nor a usable source-sidecar interpreter is available.
pub fn locate_runtime(mut report: impl FnMut(&str)) -> io::Result<SpotRuntimeCommand> {
    if let Some(path) = env::var_os(RUNTIME_OVERRIDE_ENV) {
        let path = PathBuf::from(path);
        if !path.is_file() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!(
                    "{RUNTIME_OVERRIDE_ENV} points to a missing file: {}",
                    path.display()
                ),
            ));
        }

        report("using configured runtime");
        return Ok(SpotRuntimeCommand::executable(path));
    }

    let target = RuntimeTarget::current();
    if let Ok(target) = &target {
        let path = bundled_runtime_path(&env::current_exe()?, target.filename)?;
        report(&format!("checking bundled {} runtime", target.label));
        if path.is_file() {
            report("using bundled runtime");
            return Ok(SpotRuntimeCommand::executable(path));
        }
    }

    let source_directory = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let source_sidecar = source_directory.join("spot_sidecar.py");
    if source_sidecar.is_file() {
        report("checking source-sidecar Python interpreter");
        if let Some(interpreter) = find_python_on_path(env::var_os("PATH").as_deref()) {
            let interpreter =
                prepare_source_interpreter(interpreter, &source_directory, &mut report)?;
            report("using checked-in Python sidecar");
            return Ok(SpotRuntimeCommand::python(interpreter, source_sidecar));
        }
    }

    let target = target?;
    let expected = bundled_runtime_path(&env::current_exe()?, target.filename)?;
    Err(io::Error::new(
        io::ErrorKind::NotFound,
        format!(
            "bundled Spot runtime not found at {}; source runs require Python 3.10 on PATH",
            expected.display()
        ),
    ))
}

struct RuntimeTarget {
    label: &'static str,
    filename: &'static str,
}

impl RuntimeTarget {
    fn current() -> io::Result<Self> {
        Self::for_platform(env::consts::OS, env::consts::ARCH)
    }

    fn for_platform(os: &str, arch: &str) -> io::Result<Self> {
        match (os, arch) {
            ("linux", "x86_64") => Ok(Self {
                label: "Ubuntu x86_64",
                filename: "spot-sidecar-x86_64-unknown-linux-gnu",
            }),
            ("linux", "aarch64") => Ok(Self {
                label: "Ubuntu ARM64",
                filename: "spot-sidecar-aarch64-unknown-linux-gnu",
            }),
            ("macos", "aarch64") => Ok(Self {
                label: "macOS ARM64",
                filename: "spot-sidecar-aarch64-apple-darwin",
            }),
            ("windows", "x86_64") => Ok(Self {
                label: "Windows 11 x86_64",
                filename: "spot-sidecar-x86_64-pc-windows-msvc.exe",
            }),
            _ => Err(io::Error::new(
                io::ErrorKind::Unsupported,
                format!("Spot runtime is not built for {os}/{arch}"),
            )),
        }
    }
}

fn bundled_runtime_path(executable: &Path, filename: &str) -> io::Result<PathBuf> {
    let directory = executable.parent().ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("executable path has no parent: {}", executable.display()),
        )
    })?;
    Ok(directory.join("platforms").join("spot").join(filename))
}

struct PythonInterpreter {
    program: PathBuf,
    arguments: Vec<OsString>,
}

fn prepare_source_interpreter(
    interpreter: PythonInterpreter,
    source_directory: &Path,
    report: &mut impl FnMut(&str),
) -> io::Result<PythonInterpreter> {
    if !python_is_3_10(&interpreter)? {
        return Err(io::Error::new(
            io::ErrorKind::Unsupported,
            format!(
                "{} is not Python 3.10; install Python 3.10 or use a release bundle",
                interpreter.program.display()
            ),
        ));
    }

    if python_has_runtime_dependencies(&interpreter)? {
        report("using system Python dependencies");
        return Ok(interpreter);
    }

    report("checking managed source dependencies");
    let cache_directory = source_runtime_cache_directory()?;
    fs::create_dir_all(&cache_directory)?;
    let environment_directory = cache_directory.join("python-3.10");
    let managed_python = PythonInterpreter {
        program: venv_python_path(&environment_directory, env::consts::OS),
        arguments: Vec::new(),
    };

    if !managed_python.program.is_file() || !python_is_3_10(&managed_python).unwrap_or(false) {
        report("creating managed Python environment");
        create_virtual_environment(&interpreter, &environment_directory)?;
    }

    let marker = environment_directory.join("requirements-runtime.txt");
    let requirements_match =
        fs::read_to_string(&marker).is_ok_and(|value| value == SOURCE_RUNTIME_REQUIREMENTS);
    if !requirements_match || !python_has_runtime_dependencies(&managed_python).unwrap_or(false) {
        report("installing pinned Spot SDK dependencies");
        install_runtime_dependencies(
            &managed_python,
            &source_directory.join("requirements-runtime.txt"),
        )?;
        if !python_has_runtime_dependencies(&managed_python)? {
            return Err(io::Error::other(
                "the managed Spot environment was created, but its dependency check failed",
            ));
        }
        fs::write(marker, SOURCE_RUNTIME_REQUIREMENTS)?;
    }

    report("using managed source runtime");
    Ok(managed_python)
}

fn python_is_3_10(interpreter: &PythonInterpreter) -> io::Result<bool> {
    Ok(run_python(
        interpreter,
        &[
            OsStr::new("-c"),
            OsStr::new("import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)"),
        ],
    )?
    .status
    .success())
}

fn python_has_runtime_dependencies(interpreter: &PythonInterpreter) -> io::Result<bool> {
    Ok(run_python(
        interpreter,
        &[OsStr::new("-c"), OsStr::new(SOURCE_RUNTIME_IMPORT_CHECK)],
    )?
    .status
    .success())
}

fn create_virtual_environment(
    interpreter: &PythonInterpreter,
    environment_directory: &Path,
) -> io::Result<()> {
    let output = run_python(
        interpreter,
        &[
            OsStr::new("-m"),
            OsStr::new("venv"),
            OsStr::new("--clear"),
            environment_directory.as_os_str(),
        ],
    )?;
    require_success(&output, "create the managed Spot Python environment")
}

fn install_runtime_dependencies(
    interpreter: &PythonInterpreter,
    requirements: &Path,
) -> io::Result<()> {
    let output = run_python(
        interpreter,
        &[
            OsStr::new("-m"),
            OsStr::new("pip"),
            OsStr::new("install"),
            OsStr::new("--disable-pip-version-check"),
            OsStr::new("--requirement"),
            requirements.as_os_str(),
        ],
    )?;
    require_success(&output, "install the managed Spot Python dependencies")
}

fn run_python(interpreter: &PythonInterpreter, arguments: &[&OsStr]) -> io::Result<Output> {
    Command::new(&interpreter.program)
        .args(&interpreter.arguments)
        .args(arguments)
        .output()
}

fn require_success(output: &Output, operation: &str) -> io::Result<()> {
    if output.status.success() {
        return Ok(());
    }

    let detail = String::from_utf8_lossy(&output.stderr);
    Err(io::Error::other(format!(
        "failed to {operation}: {}",
        detail.trim()
    )))
}

fn source_runtime_cache_directory() -> io::Result<PathBuf> {
    cache_base_directory(
        env::consts::OS,
        env::var_os("LOCALAPPDATA").as_deref(),
        env::var_os("XDG_CACHE_HOME").as_deref(),
        env::var_os("HOME").as_deref(),
    )
    .map(|base| base.join("dca").join("spot"))
    .ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::NotFound,
            "cannot locate a user cache directory for the managed Spot runtime",
        )
    })
}

fn cache_base_directory(
    operating_system: &str,
    local_app_data: Option<&OsStr>,
    xdg_cache_home: Option<&OsStr>,
    home: Option<&OsStr>,
) -> Option<PathBuf> {
    if operating_system == "windows" {
        return local_app_data.map(PathBuf::from);
    }
    if operating_system == "macos" {
        return home.map(|path| PathBuf::from(path).join("Library").join("Caches"));
    }
    xdg_cache_home
        .map(PathBuf::from)
        .or_else(|| home.map(|path| PathBuf::from(path).join(".cache")))
}

fn venv_python_path(environment_directory: &Path, operating_system: &str) -> PathBuf {
    if operating_system == "windows" {
        environment_directory.join("Scripts").join("python.exe")
    } else {
        environment_directory.join("bin").join("python")
    }
}

fn find_python_on_path(path: Option<&OsStr>) -> Option<PythonInterpreter> {
    let directories = env::split_paths(path?).collect::<Vec<_>>();
    for (filename, arguments) in python_candidates() {
        for directory in &directories {
            let program = directory.join(filename);
            if program.is_file() {
                return Some(PythonInterpreter {
                    program,
                    arguments: arguments.iter().map(OsString::from).collect(),
                });
            }
        }
    }
    None
}

#[cfg(windows)]
const fn python_candidates() -> &'static [(&'static str, &'static [&'static str])] {
    &[
        ("python3.10.exe", &[]),
        ("py.exe", &["-3.10"]),
        ("python3.exe", &[]),
        ("python.exe", &[]),
    ]
}

#[cfg(not(windows))]
const fn python_candidates() -> &'static [(&'static str, &'static [&'static str])] {
    &[("python3.10", &[]), ("python3", &[]), ("python", &[])]
}

#[cfg(test)]
#[path = "../../../tests/unit/platform_spot_runtime.rs"]
mod tests;
