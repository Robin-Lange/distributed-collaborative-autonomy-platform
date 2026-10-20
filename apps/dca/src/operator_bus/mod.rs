//! Shared operator protocol and bus adapter.
//!
//! This module owns the reusable operator-facing pieces that are not specific to
//! one UI surface: command text conversion, `BehaviorRequest` construction, and
//! the background bus bridge. Keeping them here prevents any single UI surface
//! from becoming the accidental owner of protocol code used by other operator
//! UIs.

pub mod command;
pub mod dds;
