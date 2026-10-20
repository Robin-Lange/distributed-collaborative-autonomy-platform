//! Standalone workspace validator used locally and by CI.
//!
//! Compile from the workspace root:
//!   rustc validate.rs -O -o validate
//!
//! Run:
//!   ./validate
//!   .\validate.exe
//!   .\validate.exe --quality-checks-only
//!   .\validate.exe --tests-only

use std::env;
use std::ffi::OsString;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

struct CommandSpec {
    name: &'static str,
    program: &'static str,
    args: &'static [&'static str],
    deny_rust_warnings: bool,
    deny_rustdoc_warnings: bool,
    capture_success_output: bool,
}

struct CheckResult {
    name: String,
    command: String,
    passed: bool,
    exit_code: Option<i32>,
    duration: Duration,
    diagnostics: String,
}

fn main() {
    let started = Instant::now();
    let workspace = match locate_workspace() {
        Ok(workspace) => workspace,
        Err(error) => {
            eprintln!("Validation could not locate the Cargo workspace: {error}");
            std::process::exit(2);
        }
    };

    let mode = match validation_mode() {
        ValidationMode::SourcePolicyOnly => {
            let result = check_authored_source_policy(&workspace);
            print_result(&result);
            if !result.passed {
                eprintln!("{}", result.diagnostics);
                std::process::exit(1);
            }
            return;
        }
        mode => mode,
    };

    let report_path = report_path(&workspace);

    println!("Rust validation");
    println!("Workspace: {}", workspace.display());
    println!("Report: {}", report_path.display());
    println!();

    let specs = command_specs(mode);
    let total = specs.len() + 1;
    let mut results = Vec::with_capacity(total);
    println!("[1/{total}] Authored-source lint policy ...");
    let policy_result = check_authored_source_policy(&workspace);
    print_result(&policy_result);
    results.push(policy_result);

    for (index, spec) in specs.iter().enumerate() {
        println!("[{}/{total}] {} ...", index + 2, spec.name);
        let result = run_command(&workspace, spec);
        print_result(&result);
        results.push(result);
    }

    let report = build_report(&workspace, started.elapsed(), &results);
    if let Err(error) = write_report(&report_path, &report) {
        eprintln!("\nValidation finished, but its report could not be written: {error}");
        std::process::exit(2);
    }

    let failures = results.iter().filter(|result| !result.passed).count();
    println!();
    if failures == 0 {
        println!("PASSED: all {} checks succeeded.", results.len());
        println!("Report written to {}", report_path.display());
        return;
    }

    println!("FAILED: {failures} of {} checks failed.", results.len());
    println!("Report written to {}", report_path.display());
    std::process::exit(1);
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ValidationMode {
    Full,
    WithoutAudit,
    QualityChecksOnly,
    TestsOnly,
    SourcePolicyOnly,
}

fn validation_mode() -> ValidationMode {
    let arguments: Vec<OsString> = env::args_os().skip(1).collect();
    match parse_validation_mode(&arguments) {
        Ok(mode) => mode,
        Err(error) => {
            eprintln!("{error}");
            std::process::exit(2);
        }
    }
}

fn parse_validation_mode(arguments: &[OsString]) -> Result<ValidationMode, String> {
    let mode = match arguments.first().map(OsString::as_os_str) {
        None => ValidationMode::Full,
        Some(argument) if argument == "--without-audit" => ValidationMode::WithoutAudit,
        Some(argument) if argument == "--quality-checks-only" => ValidationMode::QualityChecksOnly,
        Some(argument) if argument == "--tests-only" => ValidationMode::TestsOnly,
        Some(argument) if argument == "--source-policy-only" => ValidationMode::SourcePolicyOnly,
        Some(argument) => {
            return Err(format!(
                "Unknown validation option: {}",
                argument.to_string_lossy()
            ));
        }
    };
    if let Some(argument) = arguments.get(1) {
        return Err(format!(
            "Unexpected additional validation option: {}",
            argument.to_string_lossy()
        ));
    }
    Ok(mode)
}

fn command_specs(mode: ValidationMode) -> Vec<CommandSpec> {
    match mode {
        ValidationMode::SourcePolicyOnly => Vec::new(),
        ValidationMode::QualityChecksOnly => quality_check_specs(),
        ValidationMode::TestsOnly => test_specs(),
        ValidationMode::Full | ValidationMode::WithoutAudit => {
            let mut specs = quality_check_specs();
            specs.extend(test_specs());
            specs.push(release_build_spec());
            if mode == ValidationMode::Full {
                specs.push(audit_spec());
            }
            specs
        }
    }
}

fn quality_check_specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec {
            name: "Locked dependency fetch",
            program: "cargo",
            args: &["fetch", "--locked", "--quiet"],
            deny_rust_warnings: false,
            deny_rustdoc_warnings: false,
            capture_success_output: false,
        },
        CommandSpec {
            name: "Formatting",
            program: "cargo",
            args: &["fmt", "--all", "--", "--check"],
            deny_rust_warnings: false,
            deny_rustdoc_warnings: false,
            capture_success_output: false,
        },
        CommandSpec {
            name: "Compiler check",
            program: "cargo",
            args: &[
                "check",
                "--workspace",
                "--all-targets",
                "--all-features",
                "--locked",
                "--keep-going",
                "--quiet",
            ],
            deny_rust_warnings: true,
            deny_rustdoc_warnings: false,
            capture_success_output: false,
        },
        CommandSpec {
            name: "Clippy",
            program: "cargo",
            args: &[
                "clippy",
                "--workspace",
                "--all-targets",
                "--all-features",
                "--locked",
                "--keep-going",
                "--quiet",
                "--",
                "-D",
                "warnings",
                "-D",
                "clippy::all",
                "-D",
                "clippy::pedantic",
                "-D",
                "clippy::nursery",
            ],
            deny_rust_warnings: true,
            deny_rustdoc_warnings: false,
            capture_success_output: false,
        },
        CommandSpec {
            name: "Documentation build",
            program: "cargo",
            args: &[
                "doc",
                "--workspace",
                "--all-features",
                "--no-deps",
                "--locked",
                "--keep-going",
                "--quiet",
            ],
            deny_rust_warnings: true,
            deny_rustdoc_warnings: true,
            capture_success_output: false,
        },
    ]
}

fn test_specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec {
            name: "All-target tests",
            program: "cargo",
            args: &[
                "test",
                "--workspace",
                "--all-targets",
                "--all-features",
                "--locked",
                "--no-fail-fast",
                "--quiet",
            ],
            deny_rust_warnings: true,
            deny_rustdoc_warnings: true,
            capture_success_output: false,
        },
        CommandSpec {
            name: "Doctests",
            program: "cargo",
            args: &[
                "test",
                "--workspace",
                "--doc",
                "--all-features",
                "--locked",
                "--no-fail-fast",
                "--quiet",
            ],
            deny_rust_warnings: true,
            deny_rustdoc_warnings: true,
            capture_success_output: false,
        },
    ]
}

fn release_build_spec() -> CommandSpec {
    CommandSpec {
        name: "Release all-target build",
        program: "cargo",
        args: &[
            "build",
            "--release",
            "--workspace",
            "--all-targets",
            "--all-features",
            "--locked",
            "--keep-going",
            "--quiet",
        ],
        deny_rust_warnings: true,
        deny_rustdoc_warnings: false,
        capture_success_output: false,
    }
}

fn audit_spec() -> CommandSpec {
    CommandSpec {
        name: "Dependency security audit",
        program: "cargo",
        args: &["audit", "--deny", "unsound", "--deny", "yanked"],
        deny_rust_warnings: false,
        deny_rustdoc_warnings: false,
        capture_success_output: true,
    }
}

fn run_command(workspace: &Path, spec: &CommandSpec) -> CheckResult {
    let started = Instant::now();
    let mut command = Command::new(spec.program);
    command
        .args(spec.args)
        .current_dir(workspace)
        .env("CARGO_TERM_COLOR", "never");

    if spec.deny_rust_warnings {
        command.env("RUSTFLAGS", append_env_flag("RUSTFLAGS", "-D warnings"));
    }
    if spec.deny_rustdoc_warnings {
        command.env(
            "RUSTDOCFLAGS",
            append_env_flag("RUSTDOCFLAGS", "-D warnings"),
        );
    }

    match command.output() {
        Ok(output) => command_result(spec, started.elapsed(), output),
        Err(error) => CheckResult {
            name: spec.name.to_owned(),
            command: format_command(spec.program, spec.args),
            passed: false,
            exit_code: None,
            duration: started.elapsed(),
            diagnostics: format!("Unable to start command: {error}"),
        },
    }
}

fn command_result(spec: &CommandSpec, duration: Duration, output: Output) -> CheckResult {
    let passed = output.status.success();
    let raw_diagnostics = clean_diagnostics(&output.stdout, &output.stderr);
    let diagnostics = if passed && !spec.capture_success_output {
        String::new()
    } else {
        raw_diagnostics
    };

    CheckResult {
        name: spec.name.to_owned(),
        command: format_command(spec.program, spec.args),
        passed,
        exit_code: output.status.code(),
        duration,
        diagnostics,
    }
}

fn check_authored_source_policy(workspace: &Path) -> CheckResult {
    let started = Instant::now();
    let mut violations = Vec::new();
    let mut pending = vec![workspace.to_path_buf()];
    let suppression_prefixes = [
        ["#[", "allow"].concat(),
        ["#[", "expect"].concat(),
        ["#![", "allow"].concat(),
        ["#![", "expect"].concat(),
    ];

    while let Some(directory) = pending.pop() {
        let entries = match fs::read_dir(&directory) {
            Ok(entries) => entries,
            Err(error) => {
                violations.push(format!("{}: unable to read: {error}", directory.display()));
                continue;
            }
        };

        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                if !excluded_directory(&path) {
                    pending.push(path);
                }
                continue;
            }
            if path.extension().and_then(|extension| extension.to_str()) != Some("rs") {
                continue;
            }

            let contents = match fs::read_to_string(&path) {
                Ok(contents) => contents,
                Err(error) => {
                    violations.push(format!("{}: unable to read: {error}", path.display()));
                    continue;
                }
            };
            let compact_contents: String = contents
                .chars()
                .filter(|character| !character.is_whitespace())
                .collect();
            if !suppression_prefixes
                .iter()
                .any(|prefix| compact_contents.contains(prefix))
            {
                continue;
            }

            let mut found_on_one_line = false;
            for (index, line) in contents.lines().enumerate() {
                let compact_line: String = line
                    .chars()
                    .filter(|character| !character.is_whitespace())
                    .collect();
                if suppression_prefixes
                    .iter()
                    .any(|prefix| compact_line.contains(prefix))
                {
                    let relative = path.strip_prefix(workspace).unwrap_or(&path);
                    violations.push(format!(
                        "{}:{}: {}",
                        relative.display(),
                        index + 1,
                        line.trim()
                    ));
                    found_on_one_line = true;
                }
            }
            if !found_on_one_line {
                let relative = path.strip_prefix(workspace).unwrap_or(&path);
                violations.push(format!(
                    "{}: lint-suppression attribute spans multiple lines",
                    relative.display()
                ));
            }
        }
    }

    CheckResult {
        name: "Authored-source lint policy".to_owned(),
        command: "scan authored Rust for lint-suppression attributes".to_owned(),
        passed: violations.is_empty(),
        exit_code: violations.is_empty().then_some(0),
        duration: started.elapsed(),
        diagnostics: violations.join("\n"),
    }
}

fn excluded_directory(path: &Path) -> bool {
    path.file_name()
        .and_then(|name| name.to_str())
        .is_some_and(|name| matches!(name, ".cargo" | ".git" | "target" | "logs" | "third_party"))
}

fn locate_workspace() -> Result<PathBuf, String> {
    let mut starts = Vec::new();
    if let Ok(current) = env::current_dir() {
        starts.push(current);
    }
    if let Ok(executable) = env::current_exe() {
        if let Some(parent) = executable.parent() {
            if !starts.iter().any(|start| start == parent) {
                starts.push(parent.to_path_buf());
            }
        }
    }

    for start in starts {
        for ancestor in start.ancestors() {
            let manifest = ancestor.join("Cargo.toml");
            if manifest.is_file()
                && fs::read_to_string(&manifest)
                    .is_ok_and(|contents| contents.lines().any(|line| line.trim() == "[workspace]"))
            {
                return fs::canonicalize(ancestor).map_err(|error| error.to_string());
            }
        }
    }

    Err("no Cargo workspace manifest found".to_owned())
}

fn report_path(workspace: &Path) -> PathBuf {
    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    workspace
        .join("logs")
        .join(format!("validation-{timestamp}.log"))
}

fn build_report(workspace: &Path, duration: Duration, results: &[CheckResult]) -> String {
    let failures = results.iter().filter(|result| !result.passed).count();
    let mut report = if failures == 0 {
        format!("PASSED: all {} checks succeeded.\n", results.len())
    } else {
        format!("FAILED: {failures} of {} checks failed.\n", results.len())
    };
    report.push_str(&format!("Workspace: {}\n", workspace.display()));
    report.push_str(&format!("Duration: {}\n", format_duration(duration)));

    for result in results
        .iter()
        .filter(|result| !result.passed || !result.diagnostics.trim().is_empty())
    {
        report.push_str("\n================================================================\n");
        report.push_str(&format!("CHECK: {}\n", result.name));
        report.push_str(&format!("Command: {}\n", result.command));
        report.push_str(&format!("Passed: {}\n", result.passed));
        match result.exit_code {
            Some(code) => report.push_str(&format!("Exit code: {code}\n")),
            None => report.push_str("Exit code: command did not start\n"),
        }
        report.push_str(&format!("Duration: {}\n", format_duration(result.duration)));
        if !result.diagnostics.trim().is_empty() {
            report.push_str("----------------------------------------------------------------\n");
            report.push_str(result.diagnostics.trim());
            report.push('\n');
        }
    }

    report
}

fn write_report(path: &Path, report: &str) -> io::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(path, report)
}

fn clean_diagnostics(stdout: &[u8], stderr: &[u8]) -> String {
    let stdout = normalize_newlines(String::from_utf8_lossy(stdout).trim());
    let stderr = normalize_newlines(String::from_utf8_lossy(stderr).trim());

    match (stdout.is_empty(), stderr.is_empty()) {
        (true, true) => String::new(),
        (false, true) => stdout,
        (true, false) => stderr,
        (false, false) => format!("STDOUT:\n{stdout}\n\nSTDERR:\n{stderr}"),
    }
}

fn normalize_newlines(input: &str) -> String {
    input.replace("\r\n", "\n").replace('\r', "\n")
}

fn append_env_flag(variable: &str, required: &str) -> String {
    match env::var(variable) {
        Ok(existing) if !existing.trim().is_empty() => {
            format!("{} {required}", existing.trim())
        }
        _ => required.to_owned(),
    }
}

fn format_command(program: &str, args: &[&str]) -> String {
    std::iter::once(program)
        .chain(args.iter().copied())
        .map(quote_argument)
        .collect::<Vec<_>>()
        .join(" ")
}

fn quote_argument(argument: &str) -> String {
    if !argument.is_empty()
        && argument
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"-_./:=\\".contains(&byte))
    {
        return argument.to_owned();
    }
    format!(
        "\"{}\"",
        argument.replace('\\', "\\\\").replace('"', "\\\"")
    )
}

fn print_result(result: &CheckResult) {
    let status = if result.passed { "PASS" } else { "FAIL" };
    println!("{status} ({})", format_duration(result.duration));
}

fn format_duration(duration: Duration) -> String {
    let seconds = duration.as_secs();
    let millis = duration.subsec_millis();
    if seconds < 60 {
        format!("{seconds}.{millis:03}s")
    } else {
        format!("{}m {}.{millis:03}s", seconds / 60, seconds % 60)
    }
}

#[cfg(test)]
#[path = "tests/unit/validator.rs"]
mod tests;
