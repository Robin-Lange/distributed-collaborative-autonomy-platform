# Wayland scanner dependency patch

This directory contains the source of `wayland-scanner` 0.31.10 from
crates.io. Its `quick-xml` requirement is updated:

- upstream: `0.39`
- this copy: `0.41`

RustSec advisories RUSTSEC-2026-0194 and RUSTSEC-2026-0195 affect the upstream
version. `parse.rs` uses the replacement `xml10_content` method required by
`quick-xml` 0.41. The Wayland project made the same two updates after 0.31.10,
but its main branch also contains scanner API changes that are incompatible
with the other released 0.31 Wayland crates. Remove this patch after a
compatible fixed release is published.
