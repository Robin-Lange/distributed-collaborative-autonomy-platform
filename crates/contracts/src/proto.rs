//! Build-generated protobuf bindings.
//!
//! `build.rs` generates these files from `proto/core_msgs.proto` into
//! Cargo's `OUT_DIR`. This wrapper exposes them through the stable
//! `core_msgs::proto` module without checking generated code into source control.

// Rust structs generated from the shared protobuf schema.
include!(concat!(env!("OUT_DIR"), "/core_msgs.rs"));

/// Binary schema descriptor generated alongside the protobuf bindings.
pub const FILE_DESCRIPTOR_SET: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/core_msgs_descriptor.bin"));
