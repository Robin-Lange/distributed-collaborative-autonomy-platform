//! Build script for generating Rust code from the shared protobuf schema.
//!
//! Cargo runs this file before compiling the crate. Its job is to compile
//! `proto/core_msgs.proto` into Rust types using `prost_build`, so
//! the rest of the application can use strongly typed message structs instead
//! of manually parsing protobuf data.

use std::{env, error::Error, fs, path::Path, path::PathBuf};

fn main() -> Result<(), Box<dyn Error>> {
    let out_dir = PathBuf::from(env::var("OUT_DIR")?);
    let descriptor_path = out_dir.join("core_msgs_descriptor.bin");

    prost_build::Config::new()
        .protoc_executable(protoc_bin_vendored::protoc_bin_path()?)
        .file_descriptor_set_path(&descriptor_path)
        .compile_protos(&["proto/core_msgs.proto"], &["proto/"])?;
    prepare_generated_bindings(&out_dir.join("core_msgs.rs"))?;

    println!("cargo:rerun-if-changed=proto/core_msgs.proto");
    Ok(())
}

fn prepare_generated_bindings(path: &Path) -> Result<(), Box<dyn Error>> {
    let source = fs::read_to_string(path)?;
    let mut without_comments = String::with_capacity(source.len());
    for line in source.lines() {
        if !line.trim_start().starts_with("///") {
            without_comments.push_str(line);
            without_comments.push('\n');
        }
    }

    let with_contract_attributes = without_comments
        .replace(
            "    pub fn as_str_name(&self)",
            "    #[must_use]\n    pub const fn as_str_name(&self)",
        )
        .replace(
            "    pub fn from_str_name(value: &str)",
            "    #[must_use]\n    pub fn from_str_name(value: &str)",
        );
    fs::write(path, with_contract_attributes)?;
    Ok(())
}
