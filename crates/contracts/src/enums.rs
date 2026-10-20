//! Shared numeric-code vocabulary for messages on the bus.
//!
//! The autonomy stack sends many enum-like values across DDS/protobuf. Instead
//! of serializing Rust enums directly, the wire format uses compact `u8` codes.
//! This file is the single source of truth for those codes and their readable
//! names.
//!
//! Patterns used throughout this file:
//! - `*_code(...)` converts a typed Rust enum into its numeric code.
//! - `*_from_code(...)` converts a numeric code back into a typed enum.
//! - `*_name(...)` converts a numeric code into a human-readable label.
//! - zero-argument code helpers return fixed constants used when constructing
//!   messages.
//!
//! Keeping these mappings centralized prevents different crates from silently
//! disagreeing about what a numeric value means.

/// High-level behavior requested by a waypoint intent.
///
/// This is one of the few command concepts represented as a real Rust enum in
/// the public API. It is converted into a `u8` before being placed into shared
/// message structs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Behavior {
    /// Move toward the requested waypoint.
    GoTo,

    /// Move away from the requested point.
    MoveAway,

    /// Stay at the current position.
    Hold,
}

impl Behavior {
    ///
    /// These values are part of the shared message contract. Changing them would
    /// break compatibility with any process or recording that expects the old
    /// code mapping.
    const fn code(self) -> u8 {
        match self {
            Self::GoTo => 0,
            Self::MoveAway => 1,
            Self::Hold => 2,
        }
    }

    /// Converts a wire code back into a typed behavior.
    ///
    /// Returns `None` for unknown values so callers can detect malformed or
    /// future-version messages instead of accidentally treating them as valid.
    const fn from_code(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::GoTo),
            1 => Some(Self::MoveAway),
            2 => Some(Self::Hold),
            _ => None,
        }
    }
    const fn name(self) -> &'static str {
        match self {
            Self::GoTo => "GoTo",
            Self::MoveAway => "MoveAway",
            Self::Hold => "Hold",
        }
    }
}

/// Converts a typed behavior into its shared wire code.
#[must_use]
pub const fn behavior_code(value: Behavior) -> u8 {
    value.code()
}

/// Converts a shared wire code into a typed behavior, if recognized.
#[must_use]
pub const fn behavior_from_code(value: u8) -> Option<Behavior> {
    Behavior::from_code(value)
}
#[must_use]
pub const fn behavior_label(value: Behavior) -> &'static str {
    value.name()
}

/// Generates a code-family module plus crate-visible helper functions.
///
/// Message fields opt into the helpers they use:
/// - private `u8` constants for the actual numeric values,
/// - optional zero-argument helpers that return selected constants,
/// - an optional `*_name(value)` helper for diagnostic labels.
///
/// The macro removes repetitive boilerplate while keeping each code family in
/// its own namespace. Callers can use the helper functions, but they cannot
/// directly mutate or depend on the private constants.
macro_rules! make_enum_mod {
    (
        $(#[$outer:meta])*
        $mod_name:ident {
            name: $name_helper:ident,
            $(
                $const:ident $(, $code_helper:ident)? = $val:expr => $lbl:expr,
            )*
        }
    ) => {
        $(#[$outer])*
        pub mod $mod_name {
            $(
                /// Private numeric value for this code family.
                const $const: u8 = $val;

                $(
                    /// Returns this code as part of the crate-wide API.
                    pub const fn $code_helper() -> u8 {
                        $const
                    }
                )?
            )*

            /// Converts a code from this family into a readable label.
            pub const fn name(value: u8) -> &'static str {
                match value {
                    $(
                        $const => $lbl,
                    )*
                    _ => "UNKNOWN",
                }
            }
        }

        // Re-export only the helper functions. The constants stay private so
        // other modules depend on stable accessors instead of internal names.
        $(#[$outer])*
        pub use self::$mod_name::{
            $(
                $($code_helper,)?
            )*
        };

        $(#[$outer])*
        pub const fn $name_helper(value: u8) -> &'static str {
            $mod_name::name(value)
        }
    };
    (
        $(#[$outer:meta])*
        $mod_name:ident {
            $(
                $const:ident $(, $code_helper:ident)? = $val:expr => $lbl:expr,
            )*
        }
    ) => {
        $(#[$outer])*
        pub mod $mod_name {
            $(
                /// Private numeric value for this code family.
                const $const: u8 = $val;

                $(
                    /// Returns this code as part of the crate-wide API.
                    pub const fn $code_helper() -> u8 {
                        $const
                    }
                )?
            )*
        }

        // Re-export only the helper functions. The constants stay private so
        // other modules depend on stable accessors instead of internal names.
        $(#[$outer])*
        pub use self::$mod_name::{
            $(
                $($code_helper,)?
            )*
        };
    };
}

make_enum_mod! {
    /// Components that publish raw state or platform data.
    source {
        name: source_name,
        ENVIRONMENT_RUNTIME, source_environment_runtime = 0 => "ENVIRONMENT_RUNTIME",
        SPOT_ADAPTER, source_spot_adapter = 1 => "SPOT_ADAPTER",
    }
}

make_enum_mod! {
    /// Components that can request commands.
    command_source {
        AUTONOMY, command_source_autonomy = 0 => "AUTONOMY",
        HUMAN_OPERATOR, command_source_human_operator = 1 => "HUMAN_OPERATOR",
        NONE, command_source_none = 255 => "NONE",
    }
}

make_enum_mod! {
    /// Which source currently owns authoritative agent state.
    ///
    /// `NONE` is part of the wire schema (mirrored in `core_msgs.proto` and
    /// the Python bindings) but has no current Rust producer: the onboard
    /// agent's backend is now a checked enum with no unsupported fallback,
    /// so this code is never emitted from here.
    state_authority {
        ENVIRONMENT_RUNTIME, state_authority_environment_runtime = 1 => "ENVIRONMENT_RUNTIME",
        REAL, state_authority_real = 2 => "REAL",
        NONE, state_authority_none = 255 => "NONE",
    }
}

make_enum_mod! {
    /// Which source currently owns physics updates.
    ///
    /// `NONE` is part of the wire schema (mirrored in `core_msgs.proto` and
    /// the Python bindings) but has no current Rust producer; see
    /// `state_authority::NONE` for why.
    physics_authority {
        AVIAN, physics_authority_avian = 1 => "AVIAN",
        REAL_WORLD, physics_authority_real_world = 2 => "REAL_WORLD",
        NONE, physics_authority_none = 255 => "NONE",
    }
}

make_enum_mod! {
    /// Which source currently owns collision/constraint reporting.
    collision_authority {
        AVIAN_CONTACTS, collision_authority_avian_contacts = 1 => "AVIAN_CONTACTS",
        REAL_SENSORS, collision_authority_real_sensors = 2 => "REAL_SENSORS",
        FUSED, collision_authority_fused = 3 => "FUSED",
        NONE, collision_authority_none = 255 => "NONE",
    }
}

make_enum_mod! {
    /// Compatibility mode reported in authority snapshots.
    ///
    /// `EMERGENCY_STOP` is part of the wire schema (mirrored in
    /// `core_msgs.proto` and the Python bindings) but has no current Rust
    /// producer; see `state_authority::NONE` for why.
    runtime_mode {
        SIM, runtime_mode_sim = 0 => "SIM",
        REAL, runtime_mode_real = 1 => "REAL",
        HYBRID, runtime_mode_hybrid = 2 => "HYBRID",
        EMERGENCY_STOP, runtime_mode_emergency_stop = 4 => "EMERGENCY_STOP",
    }
}

make_enum_mod! {
    /// Platform-level command types.
    command_type {
        name: command_type_name,
        TRAJECTORY = 0 => "TRAJECTORY",
        HOLD, command_type_hold = 1 => "HOLD",
        STOP, command_type_stop = 2 => "STOP",
        VELOCITY, command_type_velocity = 3 => "VELOCITY",
    }
}

make_enum_mod! {
    /// Platform command lifecycle states.
    command_status {
        name: command_status_name,
        SUCCEEDED, command_status_succeeded = 0 => "SUCCEEDED",
        ABORTED, command_status_aborted = 1 => "ABORTED",
    }
}

make_enum_mod! {
    /// Reasons a command request was rejected before execution.
    veto_reason {
        name: veto_reason_name,
        NONE, veto_reason_none = 0 => "NONE",
        STALE_STATE, veto_reason_stale_state = 1 => "STALE_STATE",
        COLLISION_STOP, veto_reason_collision_stop = 2 => "COLLISION_STOP",
        UNAUTHORIZED_SOURCE, veto_reason_unauthorized_source = 3 => "UNAUTHORIZED_SOURCE",
        NO_STATE, veto_reason_no_state = 4 => "NO_STATE",
        BAD_FRAME, veto_reason_bad_frame = 5 => "BAD_FRAME",
        ROBOT_FAULT, veto_reason_robot_fault = 6 => "ROBOT_FAULT",
        UNKNOWN_OBJECT, veto_reason_unknown_object = 7 => "UNKNOWN_OBJECT",
        OBJECT_NOT_FREE, veto_reason_object_not_free = 8 => "OBJECT_NOT_FREE",
        OBJECT_NOT_HELD, veto_reason_object_not_held = 9 => "OBJECT_NOT_HELD",
        TRAJECTORY_ACTIVE, veto_reason_trajectory_active = 10 => "TRAJECTORY_ACTIVE",
        TASK_ACTIVE, veto_reason_task_active = 11 => "TASK_ACTIVE",
        UNREACHABLE, veto_reason_unreachable = 12 => "UNREACHABLE",
        PLATFORM_DEAD, veto_reason_platform_dead = 13 => "PLATFORM_DEAD",
        SUPERSEDED, veto_reason_superseded = 14 => "SUPERSEDED",
    }
}

make_enum_mod! {
    /// What a manipulation command asks the platform to do with an object.
    manipulation_verb {
        PICK_UP, manipulation_verb_pick_up = 1 => "PICK_UP",
        DROP_OFF, manipulation_verb_drop_off = 2 => "DROP_OFF",
    }
}

make_enum_mod! {
    /// Named hand-orientation profile a manipulation command should use.
    ///
    /// The sidecar derives full hand orientation from this +
    /// `target_pose.yaw_rad` rather than carrying a raw quaternion on the
    /// yaw-only wire format.
    manipulation_grasp_profile {
        TOP_DOWN, manipulation_grasp_profile_top_down = 1 => "TOP_DOWN",
    }
}

make_enum_mod! {
    /// Lifecycle phase of one manipulation command's execution.
    manipulation_phase {
        name: manipulation_phase_name,
        UNSPECIFIED = 0 => "UNSPECIFIED",
        APPROVED = 1 => "APPROVED",
        IN_PROGRESS, manipulation_phase_in_progress = 2 => "IN_PROGRESS",
        SUCCEEDED, manipulation_phase_succeeded = 3 => "SUCCEEDED",
        FAILED, manipulation_phase_failed = 4 => "FAILED",
    }
}

make_enum_mod! {
    /// Why an approved manipulation command failed during execution.
    manipulation_failure_reason {
        name: manipulation_failure_reason_name,
        UNSPECIFIED, manipulation_failure_reason_unspecified = 0 => "UNSPECIFIED",
        SAFETY_STOP, manipulation_failure_reason_safety_stop = 1 => "SAFETY_STOP",
        NO_OBJECT_DETECTED, manipulation_failure_reason_no_object_detected = 2 => "NO_OBJECT_DETECTED",
        UNSAFE_GRIP_HEIGHT, manipulation_failure_reason_unsafe_grip_height = 3 => "UNSAFE_GRIP_HEIGHT",
    }
}

make_enum_mod! {
    /// Whether a catalog object is available to pick up or currently held.
    object_lifecycle {
        name: object_lifecycle_name,
        UNSPECIFIED = 0 => "UNSPECIFIED",
        FREE, object_lifecycle_free = 1 => "FREE",
        HELD, object_lifecycle_held = 2 => "HELD",
    }
}

make_enum_mod! {
    /// Constraint categories reported by simulation, safety, or platform systems.
    constraint_type {
        name: constraint_type_name,
        WALL, constraint_type_wall = 0 => "WALL",
        ROBOT_FAULT, constraint_type_robot_fault = 1 => "ROBOT_FAULT",
        VIRTUAL_WALL, constraint_type_virtual_wall = 2 => "VIRTUAL_WALL",
        GOAL_BLOCKED, constraint_type_goal_blocked = 3 => "GOAL_BLOCKED",
    }
}

make_enum_mod! {
    /// Severity levels for constraints.
    severity {
        NONE, severity_none = 0 => "NONE",
        HARD_STOP, severity_hard_stop = 1 => "HARD_STOP",
    }
}
