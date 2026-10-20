# Zenoh transport dependency patch

This directory contains the source of `zenoh-transport` 1.9.0 from crates.io.
The only functional change is its `lz4_flex` requirement:

- upstream: `0.10.0`
- this copy: `0.11.6`

RustSec advisory RUSTSEC-2026-0041 affects the upstream version. Zenoh keeps
the dependency active even when transport compression is disabled, so Cargo
cannot remove it through feature selection. Remove this patch after Zenoh
publishes a compatible fixed release.
