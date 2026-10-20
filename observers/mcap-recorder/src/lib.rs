//! Entry point for the MCAP recorder process.
//!
//! The recorder subscribes to every shared bus topic, preserves protobuf payloads
//! as native MCAP protobuf channels, and adds a Foxglove `FrameTransform` channel
//! for convenient visualization of agent pose. Recording stops cleanly on Ctrl-C.

mod dds;
mod frame_transform;
mod record;
mod writer;

use self::dds::spawn_recorder_dds_io;
use self::writer::write_mcap;
use crossbeam_channel::bounded;
use dds_support::DDS_CHANNEL_CAPACITY;
use std::{
    error::Error,
    ffi::OsString,
    io::{Error as IoError, ErrorKind},
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};

/// Starts the DDS recorder thread and writes the MCAP file.
///
/// # Errors
///
/// Returns an error when arguments are invalid, shutdown handling cannot be
/// installed, or the recording file cannot be created or written.
pub fn run_with_args(args: impl IntoIterator<Item = OsString>) -> Result<(), Box<dyn Error>> {
    let args = RecorderArgs::from_args(args)?;
    let running = Arc::new(AtomicBool::new(true));
    install_shutdown_handler(Arc::clone(&running))?;

    let (sample_tx, sample_rx) = bounded(DDS_CHANNEL_CAPACITY);
    spawn_recorder_dds_io(sample_tx, Arc::clone(&running));

    write_mcap(&args.output_path, &sample_rx, &running)
}

/// Command-line arguments accepted by the recorder.
struct RecorderArgs {
    /// Destination MCAP path.
    output_path: PathBuf,
}

impl RecorderArgs {
    /// Parses `--out <path>` from the provided process arguments.
    fn from_args(args: impl IntoIterator<Item = OsString>) -> Result<Self, Box<dyn Error>> {
        let mut output_path = None;
        let mut args = args.into_iter();

        while let Some(arg) = args.next() {
            match arg.to_string_lossy().as_ref() {
                "--out" => output_path = Some(next_output_path(&mut args)?),
                "-h" | "--help" => return Err(invalid_input(usage()).into()),
                other => {
                    return Err(invalid_input(format!(
                        "unexpected argument `{other}`\n{}",
                        usage()
                    ))
                    .into());
                }
            }
        }

        Ok(Self {
            output_path: output_path.unwrap_or_else(default_output_path),
        })
    }
}

/// Reads the path following `--out` and reports a usage error if absent.
fn next_output_path(args: &mut impl Iterator<Item = OsString>) -> Result<PathBuf, Box<dyn Error>> {
    args.next().map_or_else(
        || Err(invalid_input(format!("missing value for `--out`\n{}", usage())).into()),
        |path| Ok(PathBuf::from(path)),
    )
}

/// Builds a timestamped default path under `logs/mcap/`.
fn default_output_path() -> PathBuf {
    let stamp = chrono::Local::now().format("%Y-%m-%d_%H-%M-%S");
    PathBuf::from(format!("logs/mcap/run_{stamp}.mcap"))
}

/// Creates a standard invalid-input error for argument parsing failures.
fn invalid_input(message: String) -> IoError {
    IoError::new(ErrorKind::InvalidInput, message)
}

/// Returns the recorder usage string.
fn usage() -> String {
    "usage: cargo run -- --component mcap_recorder [--out <path>]\n\
     without --out, records to logs/mcap/run_<YYYY-MM-DD_HH-MM-SS>.mcap"
        .to_owned()
}

/// Installs a Ctrl-C handler that asks the writer loop to finish.
fn install_shutdown_handler(running: Arc<AtomicBool>) -> Result<(), ctrlc::Error> {
    ctrlc::set_handler(move || {
        running.store(false, Ordering::SeqCst);
    })
}
