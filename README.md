# Distributed Collaborative Autonomy Platform

This repository contains a decentralized robot autonomy stack written primarily in Rust. A local launcher starts a terminal operator interface and the processes required by each configured robot. Those processes exchange agent-scoped messages over Zenoh; there is no fleet-wide controller that owns every robot.

The current platform integration is Boston Dynamics Spot. Agents can use one of three backends:

| Backend | State and execution | Virtual environment |
|---|---|---|
| `simulated` | Bevy and Avian | Owns physics, objects, and collisions |
| `hardware` | Platform adapter | Not started for that agent |
| `hybrid` | Platform adapter | Mirrors live state and evaluates virtual collisions |

Backend selection is per agent. A single launch can therefore contain simulated, hardware, and hybrid agents at the same time.

## Current scope

The operator can request five behavior primitives:

- Navigate to a planar or spatial target.
- Move away from a target.
- Hold.
- Pick an object.
- Place an object.

The platform advertises which of those operations it supports. There is no universal capability set and no silent fallback. Unsupported actions remain visible in the TUI with the reason they are disabled.

The Rust `autonomy` crate currently contains behavior-to-goal transformations and guidance. A general strategy layer and pluggable planners are not implemented yet. They can be added as local or remote producers of the same typed behavior requests used by the operator.

## Requirements

For simulation and the Rust tools:

- A current stable Rust toolchain with Rust 2024 edition support.
- Native libraries required by Bevy on the target operating system.

For a hardware-backed Spot agent:

- Network access to Spot.
- `BOSDYN_CLIENT_PASSWORD`, or a password entered through the TUI.
- A MOCAP stream when calibration or tracked objects require one.
- The Spot controller may remain connected as the E-stop endpoint, but its Robot
  Body Control must be set to `No Control` before DCA starts.

A release-bundle user does not install Python or the Boston Dynamics SDK. CI builds a standalone Spot sidecar for each supported host with Python 3.10, Boston Dynamics SDK 5.1.4, Zenoh, NumPy, and protobuf. Each release artifact contains `dca` and its matching sidecar. Simulation-only launches do not inspect or start the sidecar.

The supported matrix is Ubuntu 24.04 on x86_64 and ARM64, macOS on ARM64, and Windows 11 on x86_64.

## Run

From the repository root:

```text
cargo run --locked
```

A release bundle is also a single entry point:

```text
./dca
```

Keep the `platforms` directory next to the executable when moving or installing a release:

```text
dca-<target>/
├── dca
└── platforms/
    └── spot/
        ├── spot-sidecar-<target>
        └── spot-sidecar-<target>.sha256
```

On Windows, both executables have an `.exe` suffix.

The TUI always starts. Its setup screen controls:

- optional Bevy visualization and MCAP recording;
- MOCAP source, address, and port;
- environment dimensions;
- agent ID, platform profile, backend, maximum speed, dimensions, and initial pose;
- virtual and MOCAP-tracked objects;
- Spot connection, power-on, stand, calibration, and Motive-origin-to-body offset options.

Each navigation-capable platform defines `slow`, `medium`, and `fast`. The setup screen shows the selected profile and its actual linear and yaw-rate limits. Spot currently defines:

| Profile | Linear limit | Yaw-rate limit |
|---|---:|---:|
| Slow | 0.40 m/s | 0.50 rad/s |
| Medium | 0.90 m/s | 1.00 rad/s |
| Fast | 1.50 m/s | 1.50 rad/s |

The launcher re-enters the same Rust executable for managed Rust components. A hardware or hybrid Spot agent runs the bundled standalone sidecar as a child process. A source-tree `cargo run` requires Python 3.10. On first use, the launcher installs the packages pinned in `platforms/spot/requirements-runtime.txt` into a managed environment in the operating system's user cache. Later runs reuse that environment, and the TUI shows setup progress. Nothing is installed in the repository. Release users do not need Python and do not perform this first-use download. `DCA_SPOT_RUNTIME` can select a different standalone sidecar explicitly.

## Runtime logs

One launch uses one timestamp across its output files:

```text
logs/
├── operator_ui_<timestamp>.md
├── sidecar_<agent-id>_<timestamp>.log
└── mcap/
    └── run_<timestamp>.mcap
```

For `cargo run`, `logs` is under the repository root even if the process starts from a nested directory. For a release, it is beside the executable. The launcher does not use the shell's working directory or a home-directory fallback.

The operator log is created when the TUI starts and flushed after each event. Structured child output retains its declared `INFO`, `WARN`, or `ERROR` level; unstructured stderr remains an error. Startup exceptions are retained even when a child exits before initializing its own logger. MCAP output is created only when recording is enabled.

## Operator commands

Commands target the focused agent. Press `Tab` to change focus, or prefix a command with an agent ID.

| Command | Meaning |
|---|---|
| `g X Y` | Navigate to a map-frame point |
| `m X Y` | Move away from a map-frame point |
| `h` | Hold |
| `p [OBJECT_ID\|MOCAP_ID]` | Pick an object |
| `d [OBJECT_ID]` | Place an object; without an ID, use the object currently held |
| `r` | Resume autonomy after an operator override |

Coordinates use canonical ENU `map`-frame order: `X` is east/right and `Y` is north/forward.

An operator request pauses autonomy for that agent. Autonomy remains paused until the operator sends `r`; completion of the operator behavior does not resume it automatically.

## Control path

The main path is:

```text
operator or planner
  -> BehaviorRequest
  -> local agent: validation, authority, capabilities, safety
  -> autonomy guidance
  -> ControlRequest
  -> platform adapter or Avian simulation
```

The agent is the local policy and coordination boundary for one robot. It owns behavior lifecycle, operator override, capability admission, state freshness, constraints, and the conversion from an approved behavior to a generic platform control. The platform adapter executes that control and reports state; it does not make mission decisions.

See [OVERVIEW.md](OVERVIEW.md) for the architectural boundaries and [DOCUMENTATION.md](DOCUMENTATION.md) for an implementation map.

## Zenoh on macOS

Zenoh multicast scouting uses its automatic interface selection by default. If the host cannot initialize multicast, both Rust components and the Spot sidecar retry once with multicast disabled.

The interface can be selected explicitly:

```text
DECENTRALIZED_AUTONOMY_MULTICAST_IFACE=en0 cargo run --locked
```

Multicast can also be disabled from the start:

```text
DECENTRALIZED_AUTONOMY_MULTICAST_IFACE=disabled cargo run --locked
```

Use the same value for every process participating in a deployment. `DECENTRALIZED_AUTONOMY_DDS_NAMESPACE` can be set to isolate deployments that share a network.

## Validation

The repository validator runs the same Rust commands used for local acceptance, including the strict source policy, documentation, tests, release build, and dependency audit:

```text
rustc validate.rs -O -o target/validate
./target/validate
```

The source policy rejects authored `allow` and `expect` lint attributes. Clippy runs with `warnings`, `all`, `pedantic`, and `nursery` denied. GitLab repeats that policy and command matrix on Ubuntu x86_64, Ubuntu ARM64, macOS ARM64, and Windows x86_64. Windows divides non-release validation between required quality and test jobs because the combined cold workload does not fit reliably inside one controlled script window. Its final release job performs the same strict all-workspace, all-target, all-feature release build before packaging that executable. This preserves every check without compiling the release profile twice. The Windows CI runner uses a Windows 10-compatible baseline; the resulting executable remains compatible with the supported Windows 11 deployment.

CI prints concise Rust check progress instead of Cargo download and compilation output. The pipeline has three strict stages: validation and audit, native Spot runtime packaging, and final release compilation and packaging. A later stage starts only after every job in the earlier stages passes. Every job has a 60-minute limit; its script is limited to 50 minutes and cleanup to 5 minutes so GitLab retains time to upload failure diagnostics. Windows validation also streams each check marker to the job trace and partial validation log. Windows does not archive a Cargo cache; Unix caches retain crate archives and index metadata but omit extracted sources.

Successful test and audit jobs publish no artifact. A successful Spot runtime job publishes only its runtime and checksum, and a successful release job publishes only a named bundle containing `dca`, the matching runtime, and its checksum. Failed runtime and release jobs remove partial products before uploading their applicable preflight, validation, build, packaging, or audit diagnostics. Operator logs, sidecar session logs, and MCAP recordings are never included. Container preparation failures and exhaustion of the full job timeout remain available only in GitLab's job trace because the job cannot reliably prepare an artifact.

Run the hermetic Spot sidecar tests with Python 3.10:

```text
python3.10 -m unittest discover -s tests/python -p 'test_*.py' -v
```

The Python tests provide fake Zenoh and Boston Dynamics modules; they do not require a robot or installed SDK.

## Repository map

| Path | Responsibility |
|---|---|
| `apps/dca` | Launcher, process supervision, operator bus, Ratatui UI |
| `crates/contracts` | Shared behavior, capability, configuration, state, and wire contracts |
| `crates/autonomy` | Platform-agnostic behavior and guidance logic |
| `crates/agent` | One robot's admission, authority, safety, lifecycle, and control loop |
| `crates/middleware-zenoh` | Zenoh sessions, codecs, publishers, subscribers, and bounded channels |
| `platforms/spot` | Spot capabilities, sidecar source, runtime selection, and release packaging |
| `simulation/avian` | Shared simulated world and hybrid virtual constraints |
| `sensors/mocap` | NatNet ingest and tracked-object state |
| `observers/visualizer` | Read-only Bevy visualizer |
| `observers/mcap-recorder` | Read-only MCAP recorder |
| `third_party/zenoh-transport` | Documented Zenoh 1.9.0 security dependency patch |
| `third_party/wayland-scanner` | Documented Wayland 0.31.10 security dependency patch |
| `tests` | Dedicated Rust and Python tests |

MOCAP setup is documented separately in [MOCAP.md](MOCAP.md).
#   d i s t r i b u t e d - c o l l a b o r a t i v e - a u t o n o m y - p l a t f o r m  
 