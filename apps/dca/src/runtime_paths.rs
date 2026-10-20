//! Stable output paths for one launcher run.

use chrono::Local;
use std::{
    env, fs, io,
    path::{Path, PathBuf},
};

pub const SIDECAR_LOG_DIR_ENV: &str = "DCA_SIDECAR_LOG_DIR";
pub const RUN_ID_ENV: &str = "DCA_RUN_ID";

/// Output paths shared by the operator UI and its child processes.
#[derive(Clone, Debug)]
pub struct RunPaths {
    log_directory: PathBuf,
    run_id: String,
}

impl RunPaths {
    /// Resolves the repository root for source runs or the bundle root for a
    /// distributed executable.
    pub(super) fn discover() -> io::Result<Self> {
        let executable = env::current_exe()?;
        Self::for_executable(
            &executable,
            Local::now().format("%Y%m%d_%H%M%S").to_string(),
        )
    }

    fn for_executable(executable: &Path, run_id: String) -> io::Result<Self> {
        let executable_directory = executable.parent().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("executable path has no parent: {}", executable.display()),
            )
        })?;
        let runtime_root = find_workspace_root(executable_directory)
            .unwrap_or_else(|| executable_directory.to_path_buf());

        Ok(Self {
            log_directory: runtime_root.join("logs"),
            run_id,
        })
    }

    /// Creates every directory required by enabled log producers.
    pub(super) fn prepare(&self) -> io::Result<()> {
        fs::create_dir_all(self.log_directory.join("mcap"))
    }

    pub(super) fn log_directory(&self) -> &Path {
        &self.log_directory
    }

    pub(super) fn run_id(&self) -> &str {
        &self.run_id
    }

    pub(super) fn operator_log_path(&self) -> PathBuf {
        self.log_directory
            .join(format!("operator_ui_{}.md", self.run_id))
    }

    pub(super) fn mcap_path(&self) -> PathBuf {
        self.log_directory
            .join("mcap")
            .join(format!("run_{}.mcap", self.run_id))
    }
}

fn find_workspace_root(start: &Path) -> Option<PathBuf> {
    start
        .ancestors()
        .find(|directory| {
            directory.join("Cargo.toml").is_file()
                && directory.join("apps").join("dca").is_dir()
                && directory.join("platforms").join("spot").is_dir()
        })
        .map(Path::to_path_buf)
}

#[cfg(test)]
#[path = "../../../tests/unit/runtime_paths.rs"]
mod tests;
