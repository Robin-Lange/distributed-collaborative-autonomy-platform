# Implementation reference

This document maps the architecture to the current source tree. It describes code that exists; proposed strategy and planner work is called out separately in [OVERVIEW.md](OVERVIEW.md).

## Workspace

The repository is a Cargo workspace with one user-facing executable:

| Package | Source | Role |
|---|---|---|
| `dca` | `apps/dca` | Launcher and Ratatui operator UI |
| `dca-contracts` | `crates/contracts` | Shared contracts and protobuf generation |
| `dca-autonomy` | `crates/autonomy` | Behavior and guidance logic |
| `dca-agent` | `crates/agent` | Per-agent control runtime |
| `dca-middleware-zenoh` | `crates/middleware-zenoh` | Zenoh transport |
| `dca-platform-spot` | `platforms/spot` | Spot capability profile |
| `dca-simulation-avian` | `simulation/avian` | Simulation and hybrid world |
| `dca-sensor-mocap` | `sensors/mocap` | NatNet bridge and object tracker |
| `dca-visualizer` | `observers/visualizer` | Read-only 3D observer |
| `dca-mcap-recorder` | `observers/mcap-recorder` | Read-only log recorder |

Common package metadata and third-party versions are declared once in the root `Cargo.toml`. `Cargo.lock` is committed and release, CI, and validation commands use `--locked`.

Two dependency security fixes are pinned in the root manifest. `third_party/zenoh-transport` is Zenoh 1.9.0 with only its affected `lz4_flex` requirement updated. `third_party/wayland-scanner` is the released 0.31.10 scanner with its affected `quick-xml` requirement and the corresponding parser call updated. Each patch should be removed when its fix appears in a compatible crates.io release.

## Launcher and process supervision

`apps/dca/src/main.rs` is the executable entry point.

A normal invocation accepts no runtime flags and starts `operator_ui::run_interactive`. Internal child re-entry uses:

```text
--component environment_sim
--component agent
--component visualizer
--component mcap_recorder
--component mocap_zenoh_bridge
--component object_tracker
--component spot_sidecar
```

These component arguments are an implementation detail used by `ManagedChildren`.

`spawn_runtime_children` examines all agent manifests:

- it starts one agent process for each configured agent;
- it starts the shared environment when at least one agent is simulated or hybrid;
- it starts the object tracker for hardware or hybrid operation;
- it starts one Spot sidecar for each hardware-backed Spot agent;
- it starts the visualizer and recorder only when selected.

The TUI owns the MOCAP bridge during connection setup so it can show bridge progress and retry a timed-out stream.

Required runtime processes are the environment when selected, every agent, the object tracker when selected, and each Spot sidecar. An unexpected exit moves the connecting screen to a non-retryable startup failure and shows the process reason. Visualizer and recorder exits are reported but do not prevent robot operation.

Spot shutdown is signaled through the supervised process chain by closing its control input. This works on every supported host and gives the sidecar time to power off safely and return its lease. Other Unix children receive a termination signal. Any child that exceeds its component-specific grace period is killed.

`apps/dca/src/runtime_paths.rs` selects one output root per launcher run. Source runs use the Cargo workspace root. Release runs use the directory containing the executable. The launcher creates `logs` and `logs/mcap`, assigns one run timestamp, and passes the resolved paths to child processes. It does not derive output paths from the shell's working directory.

## Application configuration

`crates/contracts/src/config.rs` defines the complete `AppConfig`. The launcher constructs this value in memory from the setup screen and passes the same serialized JSON through `DCA_APP_CONFIG_JSON` to every child.

Each `AgentConfig` contains:

- runtime identity and guidance tuning;
- deployment manifest;
- body dimensions;
- initial simulation pose.

The deployment manifest includes:

- `agent_id`;
- platform profile;
- `AgentBackend`;
- selected `SpeedPreset`;
- command authority;
- state timeout;
- environmental-constraint trust;
- whether autonomy may command the agent.

Backend is per agent. There is no process-global runtime mode in configuration.

## Shared contracts

`crates/contracts` is the only shared vocabulary layer. Downstream packages should import its types instead of defining local copies.

### Typed behavior contract

`behavior.rs` defines:

- `BehaviorRequest`;
- `BehaviorPrimitive`;
- `PositionTarget`;
- `BehaviorStatus`;
- `ResumeAutonomy`;
- generic `ControlRequest` and `ControlStatus`.

Behavior requests and generic controls use serde JSON on Zenoh. These types preserve enum variants and target dimensionality without assigning protobuf integer codes to the new control boundary.

The agent currently translates an accepted behavior into the existing controller-specific `WaypointIntent` or `ManipulationIntent` internally. It publishes both the generic control and existing protobuf observability messages while executors consume the generic `ControlRequest`. This keeps the external platform boundary generic while current telemetry and recordings remain readable.

### Capabilities

`capabilities.rs` defines:

- `CapabilitySet`;
- `CapabilityRequirement`;
- `CapabilityReport`;
- `NavigationCapability`;
- `ManipulationCapability`;
- `SpeedPreset`, `SpeedPresets`, and `SpeedLimits`.

An empty capability set supports nothing. `CapabilityReport::require` checks live connectivity separately from static support.

The launcher obtains a first-party profile from `apps/dca/src/platform_profiles.rs`, serializes it through `DCA_AGENT_CAPABILITIES_JSON`, and starts the agent. `crates/agent/src/config.rs` rejects non-finite, zero, or negative navigation limits at the process boundary.

### Protobuf contract

`crates/contracts/proto/core_msgs.proto` contains state, constraint, compatibility command, MOCAP, manipulation, and object messages. `build.rs` uses a vendored `protoc`, so contributors do not need a system protobuf compiler.

Rust conversion code is in `crates/contracts/src/convert.rs`. The generated Python file used by Spot is checked in at `platforms/spot/core_msgs_pb2.py`.

### Frames and objects

`frames.rs` defines rigid transforms and the static registrations:

- Bevy world to ENU `map`;
- MOCAP lab to ENU `map`.

The naming rule is `T_parent_child`: the transform maps a point expressed in the child frame into the parent frame.

`catalog.rs` loads virtual and MOCAP-tracked object definitions. `manipulation.rs` owns reach and manipulation precondition evaluation.

### Constraints

`constraints.rs` stores active hard stops by `(source, constraint type, object id)`.

- Environmental stops expire when they are not refreshed.
- Platform faults remain until explicitly cleared.
- Faults outrank environmental contacts.
- A newly accepted navigation request removes only older goal-blocked diagnostics, not unrelated constraints.

`ActiveConstraints::has_environment_wall_stop` supports collision recovery. The runtime can retain a new request while a previous wall event clears.

## Agent runtime

`crates/agent/src/lib.rs` selects one configured agent using `DCA_AGENT_ID`, loads its capabilities and object catalog, starts the bus adapter, and enters `runtime::run`.

### Admission

`behavior_gate.rs` performs structural, identity, frame, authority, capability, target-heading, and availability checks.

Operator requests latch the operator override before capability or availability checks. Autonomy remains paused until a matching `ResumeAutonomy` command arrives.

### Runtime ownership

`runtime.rs` owns the mutable state for one agent:

- active typed behavior;
- current controller intent;
- latest accepted platform state;
- platform liveliness;
- active constraints;
- manipulation task;
- behavior and command status.

It emits `Accepted` and `Active` when a request enters execution, then `Succeeded`, `Failed`, `Rejected`, `Cancelled`, or `Preempted` as appropriate.

Manipulation has an additional completion check. A successful executor status remains pending until the matching object state confirms the effect: held by the same agent after Pick, or free with no holder after Place. If the lifecycle does not confirm within two seconds, the behavior fails instead of publishing a false success.

The runtime routes primitives as follows:

| Primitive | Internal execution path |
|---|---|
| `NavigateTo` | Navigation intent toward the supplied target |
| `MoveAway` | Navigation intent derived from the supplied point and configured standoff |
| `Hold` | Immediate hold intent |
| `Pick` | Manipulation preconditions, then pick control |
| `Place` | Ownership and reach checks, then place control |

### Authority and constraints

`authority.rs` maps each `AgentBackend` to accepted state and collision sources:

- simulated accepts Avian state and optionally trusts Avian contacts;
- hardware accepts platform state and trusts platform faults;
- hybrid accepts platform state and trusts platform faults plus Avian virtual contacts.

`decision.rs` checks state presence, freshness, platform availability, authority, and active hard stops before publishing controls. Platform speed limits selected in the manifest become `GuidanceLimits`.

When an environmental collision stopped the previous command, a newly accepted navigation behavior remains pending until that level-published collision clears or expires. Robot-fault stops do not use this path.

### Guidance

`crates/autonomy/src/behaviors.rs` contains behavior-to-goal transformations. `guidance.rs` contains the planar velocity controller and tracking outcome.

Guidance receives plain data and returns plain actions. It does not read environment variables, publish messages, or import platform code.

The controller:

- computes map-frame position and heading error;
- rotates the planar error into the flat-body frame;
- applies a proportional law;
- clamps linear speed and yaw rate to the selected platform profile;
- reports completion within the platform's position tolerance.

## Platform execution

### Spot

`platforms/spot/src/lib.rs` defines Spot's capabilities. `platforms/spot/src/runtime.rs` selects the standalone SDK sidecar for the current host.

Release CI uses `platforms/spot/build_runtime.py` and `requirements-build.txt` to build one PyInstaller executable on each native target. The runtime contains Python 3.10, Boston Dynamics SDK 5.1.4 and its package resources, Zenoh, NumPy, protobuf, `spot_sidecar.py`, and `core_msgs_pb2.py`. The macOS job installs those build dependencies in a job-local Python 3.10 virtual environment, leaving the runner's managed Python installation unchanged. The build runs a packaged-runtime self-test that creates a standard Boston Dynamics client before accepting the artifact. This exercises the SDK's dynamic resource imports instead of checking only argument parsing. PyInstaller is not used as a cross-compiler; each operating-system and architecture artifact is built on its matching runner.

When a Spot child starts, the launcher:

1. uses `DCA_SPOT_RUNTIME` when an explicit executable is configured;
2. otherwise selects the native sidecar under `platforms/spot` beside the release executable;
3. when running from a source checkout, verifies Python 3.10 and its Spot runtime dependencies;
4. if those dependencies are absent, creates a managed environment in the operating system's user cache and installs the versions pinned in `requirements-runtime.txt`;
5. starts the selected runtime as a supervised child.

The TUI shows runtime selection and first-use installation progress, and retains the last sidecar error on the connection-failure screen. A simulation-only launch never invokes this path. Credentials are passed only in the sidecar child environment.

The sidecar acquires Spot's body lease but never forcefully takes it from another controller. The Spot controller may remain connected as the E-stop endpoint, but Robot Body Control must be set to `No Control` before DCA starts. A lease conflict reports the current SDK owner when Spot provides it. Lease-retain failures report their status and current owner while the SDK retries. An active Keepalive policy that requests motors off is reported separately and is not removed by DCA. The owning tablet, mission, AutoReturn session, or SDK client must release those controls. On shutdown or a startup failure after acquisition, the sidecar safely powers off when necessary, stops its lease keepalive, and requests lease return before exiting.

Release artifacts preserve this layout:

```text
dca-<target>/
├── dca
└── platforms/
    └── spot/
        ├── spot-sidecar-<target>
        └── spot-sidecar-<target>.sha256
```

The checksum is distributed for artifact verification; a release launcher does not fetch files at runtime. Source-tree hardware development requires Python 3.10 and downloads the pinned runtime packages once when they are missing. The managed environment is stored under `%LOCALAPPDATA%\dca\spot` on Windows, `~/Library/Caches/dca/spot` on macOS, and `$XDG_CACHE_HOME/dca/spot` or `~/.cache/dca/spot` on Linux. Release users do not need Python or an SDK installation.

The Python adapter:

- subscribes to generic JSON `ControlRequest` values;
- requires the matching `agent_id` and `approved_by == "onboard_agent"`;
- converts planar velocity, hold, stop, pick, and place to Boston Dynamics SDK calls;
- publishes real agent state, manipulation status, object state where applicable, frame calibration, and robot-fault constraints;
- declares state liveliness for immediate disconnect detection.

Mobility velocity and stop RPCs share one in-flight gate. A stop replaces pending velocity work and is never skipped. Arm phases execute sequentially on the manipulation worker. This preserves deterministic stop precedence without claiming that unrelated arm feedback uses the mobility gate.

Motor power state is published as a platform hard stop. Losing power therefore fails the active behavior, disables new motion through the normal safety path, and clears automatically when Spot reports the motors on again. If an SDK call races that state update and returns a motors-off error, the sidecar clears the active motion request and suppresses redispatch of that command. This prevents one stale velocity request from producing an unbounded stream of identical SDK errors.

The sidecar re-applies an SDK velocity limit even though the Rust guidance layer has already clamped the request.

For top-down pick and place, the published object position is the requested grip point. The sidecar transforms that full XYZ position from map to Spot's vision frame and commands an explicit tool frame centered between the gripper jaws. The sidecar never changes that target to make it safe: a point below Spot's configured live ground-plane clearance is rejected before the arm moves. Hip-height assistance is enabled for vertical reach; body-yaw assistance remains disabled.

Real-object pickup succeeds only when Spot reports that the gripper is applying force. A fully closed gripper means that no object was detected and produces a distinct execution failure. Unsafe target height, no detected object, target distance, and target direction are reported separately in the operator UI so that a failed request can be corrected without interpreting an SDK status code.

### Avian

`simulation/avian/src/dds.rs` subscribes to generic controls and routes them to drive or manipulation channels.

`startup.rs` creates:

- dynamic bodies for simulated agents;
- kinematic mirrors for hybrid agents;
- arena walls;
- configured virtual and tracked objects.

`systems.rs`:

- applies a first-order plant response to simulated velocities;
- publishes simulated state;
- mirrors real state into hybrid entities;
- classifies blocking contacts;
- publishes and clears environmental hard stops;
- runs simulated manipulation;
- follows a hybrid holder with continuous object-state updates.

Simulated wall contact latches a hard stop for the current physics tick, locks the body immediately, and clears only when the command no longer pushes into the contact. Hybrid bodies use sensor-only virtual geometry, so contacts report constraints without allowing Avian to displace the hardware mirror.

Collision push detection uses the requested direction relative to the contact normal. A hold or retreat does not continuously assert a pushing constraint.

## Middleware

`crates/middleware-zenoh/src/lib.rs` provides:

- session setup;
- protobuf and JSON codecs;
- publisher and subscriber declarations;
- liveliness;
- bounded event and latest-wins channels;
- rate-limited transport diagnostics.

The default Zenoh peer uses an ephemeral TCP listener and multicast interface `auto`. `DECENTRALIZED_AUTONOMY_MULTICAST_IFACE` accepts an explicit interface name or `disabled`. A failed multicast open is retried once with multicast disabled.

The Rust build enables only Zenoh's TCP and UDP transports. Authentication, compression, QUIC, TLS, WebSocket, and local-socket features are not compiled because the current transport configuration does not use them.

`DECENTRALIZED_AUTONOMY_DDS_NAMESPACE` overrides the default `decentralized_autonomy` topic prefix.

Discrete requests, statuses, and constraints are drained in arrival order. High-rate state uses bounded latest-wins queues so a slow consumer catches up to current state instead of replaying a stale backlog.

## TUI

`apps/dca/src/operator_ui.rs` contains four screens:

- setup;
- Spot password entry;
- connection progress or failure;
- operator view.

The setup screen edits the same `AppConfig` sent to all children. The operator screen subscribes to capability, state, behavior status, command, veto, constraint, manipulation, object, and process events.

Unsupported and offline controls remain visible with a reason. `Tab` changes the focused agent. A command can also begin with an agent ID for explicit targeting.

Navigation and move-away input always use `X Y` map-frame order. Each navigation-capable platform exposes slow, medium, and fast speed profiles; the setup screen shows the selected profile and its actual limits.

Terminal teardown disables mouse capture, drains pending input sequences, leaves the alternate screen, restores the cursor, and then disables raw mode. This prevents terminal mouse-report fragments from appearing before or after the UI.

The operator activity log is opened with the TUI and flushed after each recorded event. Structured child output retains its declared `INFO`, `WARN`, or `ERROR` severity even when the logging framework writes every level to stderr. Unstructured stderr remains an error. Startup failures and child warnings are retained; routine child-process information remains out of the transcript. The operator log, per-agent sidecar logs, and optional MCAP file share one run timestamp under the output root selected by `runtime_paths.rs`.

## MOCAP and observers

`sensors/mocap/src/bridge.rs` accepts NatNet 4 frames from direct UDP multicast or the TCP relay and publishes `MocapRigidBodyFrame`.

`sensors/mocap/src/tracker.rs` maps configured rigid-body IDs into `ObjectState`, applies the MOCAP-to-map transform, tracks lifecycle changes from manipulation status, and marks stale poses invalid.

`observers/visualizer` subscribes to state, object, intent, and command topics. It does not publish controls.

`observers/mcap-recorder` subscribes to the bus and writes timestamped MCAP files under `logs/`. It also writes derived frame transforms for compatible visualization tools.

## Tests

Rust test bodies live under `tests/unit` and are included by the package that owns the private code under test. This keeps production files focused while preserving access to crate-private and module-private transformations.

Python sidecar tests live under `tests/python`. `fake_sidecar_runtime.py` installs in-memory fake modules for Zenoh, NumPy, protobuf, and the Boston Dynamics SDK.

The important regressions include:

- a post-collision command is retained and executes after the old wall event clears;
- robot faults do not use environmental collision expiry;
- hybrid held objects update on every real pose;
- Pick and Place cannot succeed until object ownership confirms the executor result;
- real MOCAP-tracked objects apply only matching terminal manipulation results;
- speed selection reaches guidance limits;
- generic controls route correctly;
- Spot stop and RPC precedence;
- repeated motors-off rejections are latched per command;
- Spot runtime target selection and bundled command construction;
- the packaged Spot runtime includes the SDK resources required by `create_standard_sdk`;
- terminal and component lifecycle behavior;
- canonical operator coordinate ordering;
- simulated hard-stop latching and hybrid sensor-only collision geometry;
- continuous pick, carry, and place visualization transitions;
- Zenoh multicast default, override, and fallback behavior.

The root `validate.rs` rejects authored lint-suppression attributes, then runs locked dependency fetch, formatting, all-target compilation, strict Clippy, documentation, all-target tests, doctests, release compilation, and `cargo audit`. CI runs the platform-independent audit once and runs the remaining validation matrix on each supported Rust target. Windows uses `--quality-checks-only` and `--tests-only` in two required test-stage jobs, while its final release job performs the same strict all-workspace, all-target, all-feature release build before smoke testing and packaging that executable. No check is dropped, and the release profile is compiled only once.

The pipeline has three stage barriers. All test-stage validation and audit jobs must pass before native Spot runtimes are built, and every runtime build must pass before final release compilation and packaging begins. Each job has a 60-minute limit. Scripts are limited to 50 minutes and cleanup to 5 minutes, leaving time for GitLab to collect diagnostics after a controlled timeout. Windows validation streams each check marker to both the job trace and its partial bootstrap log. Windows Cargo caching is disabled because archiving extracted registry sources can exhaust the job limit after validation succeeds. Unix caches retain registry archives and index metadata, from which Cargo reconstructs extracted sources.

Successful tests and audits publish no artifact. A successful native-runtime job contains only the standalone runtime and checksum. A successful final-release artifact contains only the application executable, matching standalone Spot runtime, and checksum. Failed runtime and release jobs remove partial products and retain only their applicable preflight, validation, build, packaging, or audit diagnostics. Operator logs, sidecar session logs, prior validator reports, and MCAP recordings are not collected. Failures that prevent the runner from creating its container, and exhaustion of the full job timeout, remain available only in GitLab's job trace. Rust tests stay under `tests/unit`; production modules include those files only to retain access to private behavior.
