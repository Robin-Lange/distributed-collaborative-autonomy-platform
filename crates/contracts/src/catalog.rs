//! Shared object catalog: known, deterministic-pose manipulation targets.
//!
//! Manipulation uses deterministic known poses: simulated and hybrid objects can start
//! from catalog poses, while `object_tracker` supplies live MOCAP poses for
//! physical objects on hardware-backed agents. This module loads the shared catalog and the
//! reach envelope used to gate manipulation requests before any command is
//! attempted (see [`crate::manipulation`]).

use crate::{manipulation::ReachEnvelope, messages::PoseXYZYaw};
use serde::Deserialize;
use std::collections::HashMap;

/// One catalog entry: a known object with a fixed size and starting pose.
#[derive(Deserialize, Clone, Debug)]
pub struct ObjectSpec {
    object_id: String,

    /// Used for rendering and labeling; reach logic is shape-agnostic.
    shape: String,

    length_m: f32,

    /// Radius, in meters, for a `"cylinder"`-shaped object (unused —
    /// `0.0` — for `"box"`, which is fully described by `length_m` +
    /// `width_m` + `height_m`).
    #[serde(default)]
    radius_m: f32,

    /// Rendering-only color name (e.g. `"pink"`) kept independent of any
    /// specific rendering technology.
    color: String,

    /// Width and height, in meters, for a `"box"`-shaped object (unused —
    /// `0.0` — for `"cylinder"`, which is fully described by `length_m` +
    /// `radius_m`). Rendering/labeling only, like `length_m`/`radius_m`.
    #[serde(default)]
    width_m: f32,
    #[serde(default)]
    height_m: f32,

    /// Pose used as the object's sim/hybrid starting pose. `None` for a
    /// MOCAP-tracked object (`mocap_rigid_body_id` is `Some`): its
    /// location is whatever `object_tracker` observes, not a fixed spawn
    /// point — the object can be placed anywhere on the MOCAP stage, so
    /// there is no sane placeholder to spawn it at in sim/hybrid, and
    /// nothing there needs one (`object_tracker` is hardware-only).
    #[serde(default)]
    virtual_initial_pose_map: Option<PoseXYZYaw>,

    /// NatNet/Motive Streaming ID of the physical rigid body this object
    /// corresponds to, if it is a real, MOCAP-tracked object rather than a
    /// purely virtual one. `None` for virtual-only catalog entries.
    /// `object_tracker` is the sole `ObjectState` authority for hardware-tracked
    /// entries where this is
    /// `Some` — the sidecar's `--owns-object-state` seed list skips
    /// them to avoid two publishers racing on the same `object_id`.
    #[serde(default)]
    mocap_rigid_body_id: Option<i32>,
}

impl ObjectSpec {
    #[must_use]
    pub fn object_id(&self) -> &str {
        &self.object_id
    }
    #[must_use]
    pub const fn mocap_rigid_body_id(&self) -> Option<i32> {
        self.mocap_rigid_body_id
    }
    #[must_use]
    pub const fn virtual_initial_pose_map(&self) -> Option<PoseXYZYaw> {
        self.virtual_initial_pose_map
    }
    #[must_use]
    pub fn shape(&self) -> &str {
        &self.shape
    }
    #[must_use]
    pub const fn length_m(&self) -> f32 {
        self.length_m
    }
    #[must_use]
    pub const fn radius_m(&self) -> f32 {
        self.radius_m
    }
    #[must_use]
    pub const fn width_m(&self) -> f32 {
        self.width_m
    }
    #[must_use]
    pub const fn height_m(&self) -> f32 {
        self.height_m
    }
    #[must_use]
    pub fn color(&self) -> &str {
        &self.color
    }

    /// The object's own resting height (map `z`) when sitting on the floor,
    /// used e.g. as `default_drop_pose`'s placement height. Prefers the
    /// catalog spawn pose's `z` where one exists (preserves the exact,
    /// possibly hand-tuned CG height documented on virtual objects);
    /// otherwise derives it from the object's own declared `height_m`
    /// (half the height — a box centered on its own CG, flat on the floor)
    /// rather than requiring a pose a real, MOCAP-tracked object does not
    /// have.
    #[must_use]
    pub fn resting_height_m(&self) -> f32 {
        self.virtual_initial_pose_map
            .map_or(self.height_m / 2.0, |pose| pose.z)
    }
}

/// Converts the `AppConfig`-embedded, plain-DTO `ObjectSpec` (public fields,
/// (de)serializable so it can round-trip through `DCA_APP_CONFIG_JSON` and be
/// edited field-by-field from the operator UI's Setup screen) into this
/// module's private-fielded, accessor-gated domain type.
impl From<crate::config::ObjectSpec> for ObjectSpec {
    fn from(spec: crate::config::ObjectSpec) -> Self {
        Self {
            object_id: spec.object_id,
            shape: spec.shape,
            length_m: spec.length_m,
            radius_m: spec.radius_m,
            color: spec.color,
            width_m: spec.width_m,
            height_m: spec.height_m,
            virtual_initial_pose_map: spec.virtual_initial_pose_map.map(|pose| PoseXYZYaw {
                x: pose.x,
                y: pose.y,
                z: pose.z,
                yaw_rad: pose.yaw_rad,
            }),
            // NatNet streaming IDs are always small non-negative numbers;
            // saturating rather than panicking on a malformed config keeps
            // this conversion infallible.
            mocap_rigid_body_id: spec
                .mocap_rigid_body_id
                .map(|id| i32::try_from(id).unwrap_or(i32::MAX)),
        }
    }
}

/// Loaded object catalog plus the shared reach envelope.
pub struct ObjectCatalog {
    reach_envelope: ReachEnvelope,
    objects: HashMap<String, ObjectSpec>,
}

impl ObjectCatalog {
    /// Builds the catalog from an already-loaded `AppConfig.objects` block,
    /// so operator-edited entries (added via the Setup screen) reach the
    /// running catalog.
    #[must_use]
    pub fn from_config(config: crate::config::ObjectCatalogConfig) -> Self {
        let objects = config
            .objects
            .into_iter()
            .map(|spec| {
                let spec = ObjectSpec::from(spec);
                (spec.object_id().to_owned(), spec)
            })
            .collect();
        Self {
            reach_envelope: config.reach_envelope,
            objects,
        }
    }

    /// Looks up one catalog entry by object id.
    #[must_use]
    pub fn get(&self, object_id: &str) -> Option<&ObjectSpec> {
        self.objects.get(object_id)
    }

    /// Iterates every catalog entry, for startup seeding.
    pub fn iter(&self) -> impl Iterator<Item = &ObjectSpec> {
        self.objects.values()
    }
    #[must_use]
    pub const fn reach_envelope(&self) -> ReachEnvelope {
        self.reach_envelope
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/core_msgs/catalog.rs"]
mod catalog_tests;
