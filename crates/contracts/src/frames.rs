//! Canonical map frame contract and SE(3) rigid-transform utilities.
//!
//! The canonical `map` frame uses ENU convention: +X east, +Y north, +Z up,
//! right-handed, metres and radians.  Every spatial message on the bus carries
//! a `frame_id` that must equal `"map"` for the agent to act on it.

/// The bus-canonical frame name that all spatial messages must carry.
pub const MAP_FRAME: &str = "map";

/// The NatNet/Motive MOCAP frame name (Y-up, +X right, +Y up, +Z back).
pub const MOCAP_LAB_FRAME: &str = "mocap_lab";

// ── SE(3) rigid transform ──────────────────────────────────────────────────────

/// A rigid body transform in SE(3): a 3×3 rotation matrix and a 3-vector
/// translation, representing the pose of `child` expressed in `parent`.
///
/// Notation: `T_parent_child` maps points from the child frame into the parent
/// frame:  `p_parent = R * p_child + t`.
///
/// All arithmetic uses `f64` so accumulated calibration errors stay small.
#[derive(Clone, Debug)]
pub struct RigidTransform {
    /// Row-major 3×3 rotation matrix (det = +1, R^T R = I): `r[i][j]` is row
    /// `i`, column `j`, so `transform_point` computes `p_out[i] = sum_j r[i][j] * p[j]`.
    pub r: [[f64; 3]; 3],
    /// Translation vector expressed in the parent frame.
    pub t: [f64; 3],
}

impl RigidTransform {
    /// Constructs from a rotation matrix (rows) and translation.
    ///
    /// The caller is responsible for supplying a valid rotation matrix
    /// (orthonormal, det = +1).  No normalisation is applied here.
    #[must_use]
    pub const fn from_rotation_translation(r: [[f64; 3]; 3], t: [f64; 3]) -> Self {
        Self { r, t }
    }

    /// Computes the inverse transform: if `self` is `T_A_B` then the result is `T_B_A`.
    ///
    /// The rotation inverse is the exact transpose (orthonormality-preserving,
    /// matching the stored convention); only the `-Rᵀ · t` product runs through
    /// `glam`.
    #[must_use]
    pub fn inverse(&self) -> Self {
        let rt = mat3_transpose(&self.r);
        let t = (-(mat3_from_rows(&rt) * glam::DVec3::from_array(self.t))).to_array();
        Self { r: rt, t }
    }

    /// Applies the full SE(3) transform to a point: `R * p + t`.
    #[must_use]
    pub fn transform_point(&self, p: [f64; 3]) -> [f64; 3] {
        (mat3_from_rows(&self.r) * glam::DVec3::from_array(p) + glam::DVec3::from_array(self.t))
            .to_array()
    }

    /// Applies only the rotation to a free vector (no translation): `R * v`.
    #[must_use]
    pub fn transform_vector(&self, v: [f64; 3]) -> [f64; 3] {
        (mat3_from_rows(&self.r) * glam::DVec3::from_array(v)).to_array()
    }
}

// ── Quaternion validation ──────────────────────────────────────────────────────

/// Minimum squared norm below which a quaternion no longer defines a rotation.
const QUAT_MIN_NORM_SQUARED: f64 = 1e-12;

/// Returns true when every component is finite and the norm is far enough from
/// zero to define a rotation.
///
/// Callers on the calibration path must reject quaternions that fail this check
/// instead of silently treating them as identity: a degenerate MOCAP sample
/// must never lock in a wrong registration.
#[must_use]
pub fn quat_is_valid(qx: f64, qy: f64, qz: f64, qw: f64) -> bool {
    let q = glam::DQuat::from_xyzw(qx, qy, qz, qw);
    q.is_finite() && q.length_squared() > QUAT_MIN_NORM_SQUARED
}

// ── Full-orientation (6-DoF) conversion ─────────────────────────────────────────

/// Determinant of a row-major 3×3 matrix.
///
/// Exposed so callers can sanity-check a loaded `T_*_*` config entry is a
/// proper rotation (`det ≈ +1`) before trusting [`mat3_to_quat`] on it —
/// `transform_point`/`transform_vector` silently tolerate a reflected
/// (`det ≈ -1`) matrix, so a mirrored config is otherwise invisible.
#[must_use]
pub fn mat3_determinant(r: &[[f64; 3]; 3]) -> f64 {
    r[0][2].mul_add(
        r[1][1].mul_add(-r[2][0], r[1][0] * r[2][1]),
        r[0][1].mul_add(
            -r[1][2].mul_add(-r[2][0], r[1][0] * r[2][2]),
            r[0][0] * r[1][2].mul_add(-r[2][1], r[1][1] * r[2][2]),
        ),
    )
}

/// Converts a row-major 3×3 rotation matrix into a quaternion.
///
/// The matrix is stored on [`RigidTransform::r`]. The result `(x, y, z, w)`
/// supports composing a
/// tracked rigid body's *full* orientation across frames (unlike
/// extracting Spot's yaw only, under Spot's own `R_spot_rb` mounting
/// convention — not applicable to a generic tracked object).
///
/// Debug-asserts the input is a proper rotation (`det ≈ +1`): a reflected
/// matrix (a mirrored MOCAP/Bevy axis convention in a config file) would
/// make `glam`'s conversion produce a quaternion that does not represent the
/// same transform as `r`, silently. `mat3_determinant` is exposed separately
/// so release builds can still log the same check.
#[must_use]
pub fn mat3_to_quat(r: &[[f64; 3]; 3]) -> (f64, f64, f64, f64) {
    debug_assert!(
        (mat3_determinant(r) - 1.0).abs() < 1e-3,
        "mat3_to_quat: input is not a proper rotation (det = {})",
        mat3_determinant(r)
    );

    // `glam::DMat3::from_cols` takes column vectors; `r[i][j]` is row i,
    // column j, so column `j` is `(r[0][j], r[1][j], r[2][j])`.
    let mat = glam::DMat3::from_cols(
        glam::DVec3::new(r[0][0], r[1][0], r[2][0]),
        glam::DVec3::new(r[0][1], r[1][1], r[2][1]),
        glam::DVec3::new(r[0][2], r[1][2], r[2][2]),
    );
    let q = glam::DQuat::from_mat3(&mat).normalize();
    (q.x, q.y, q.z, q.w)
}

// ── Static configuration (frame registrations) ──────────────────────────────

/// Static per-deployment frame transforms.
///
/// Built from `crate::config::FramesConfig` (the `AppConfig`
/// section carrying the raw rotation/translation entries) via [`Self::from_config`]
/// — this type is that data's converted, transform-ready form.
pub struct FramesConfig {
    /// `T_map_bevy`: maps Bevy/world-frame points into `map`.
    pub t_map_bevy: RigidTransform,
    /// `T_map_mocap`: maps raw NatNet/Motive points into `map`.
    pub t_map_mocap: RigidTransform,
}

impl FramesConfig {
    /// Converts an already-loaded `AppConfig.frames` block.
    #[must_use]
    pub const fn from_config(config: &crate::config::FramesConfig) -> Self {
        Self {
            t_map_bevy: Self::entry_to_transform(&config.frames.bevy_world_to_map),
            t_map_mocap: Self::entry_to_transform(&config.frames.mocap_lab_to_map),
        }
    }

    const fn entry_to_transform(entry: &crate::config::FrameEntryJson) -> RigidTransform {
        RigidTransform::from_rotation_translation(entry.rotation_matrix_rows, entry.translation_m)
    }
}

// ── Internal helpers ───────────────────────────────────────────────────────────

/// Builds a column-major [`glam::DMat3`] from a row-major 3×3 (`r[i][j]` is row
/// `i`, column `j`), so `mat3_from_rows(r) * v` yields the same `R · v` the rows
/// encode. Mirrors the column construction used by [`mat3_to_quat`].
const fn mat3_from_rows(r: &[[f64; 3]; 3]) -> glam::DMat3 {
    glam::DMat3::from_cols(
        glam::DVec3::new(r[0][0], r[1][0], r[2][0]),
        glam::DVec3::new(r[0][1], r[1][1], r[2][1]),
        glam::DVec3::new(r[0][2], r[1][2], r[2][2]),
    )
}

const fn mat3_transpose(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

#[cfg(test)]
#[path = "../../../tests/unit/core_msgs/frames.rs"]
mod frames_tests;
