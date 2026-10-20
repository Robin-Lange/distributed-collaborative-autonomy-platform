//! Startup systems that create the visualizer's render scene.
//!
//! These systems run once during Bevy startup. They create only visual entities:
//! ground, translucent walls, the camera, HUD text, a diagnostic obstacle, the
//! agent mesh, and colored world-axis datum markers. Runtime pose updates happen in
//! `systems.rs`.

use super::{
    config::VisualizerConfig,
    systems::{forward_facing_rotation, object_rotation},
    world::{
        FrameConversion, HudText, IDLE_HUD_TEXT, ObjectCatalogRes, ObjectMotion, VisualAgent,
        VisualObject, VisualObjects,
    },
};
use bevy::prelude::*;
use core_msgs::PoseXYZYaw;

/// HUD font size in pixels.
const HUD_FONT_SIZE: f32 = 18.0;
/// HUD left offset from the window edge in pixels.
const HUD_LEFT_PX: f32 = 12.0;
/// HUD top offset from the window edge in pixels.
const HUD_TOP_PX: f32 = 12.0;
/// Ambient light brightness used to keep the scene readable without extra lights.
const AMBIENT_LIGHT_BRIGHTNESS: f32 = 800.0;
/// Length of each rendered datum axis arrow.
const DATUM_AXIS_LENGTH: f32 = 0.3;
/// Height used for rendered boundary walls; this is visual-only.
const WALL_RENDER_HEIGHT: f32 = 0.5;

/// Ground-plane material color.
const GROUND_COLOR: Color = Color::srgb(0.3, 0.5, 0.3);
/// Semi-transparent wall color so the operator can see through the arena bounds.
const WALL_COLOR: Color = Color::srgba(0.05, 0.2, 0.05, 0.3);
/// Semi-transparent body color for the observed agent.
const AGENT_COLOR: Color = Color::srgba(251.0 / 255.0, 212.0 / 255.0, 3.0 / 255.0, 0.85);
/// Arm marker color — black, distinct from the translucent yellow body.
const ARM_MARKER_COLOR: Color = Color::srgb(0.05, 0.05, 0.05);
/// Arm dimensions. Boston Dynamics specifies 984 mm at full extension.
const ARM_LENGTH_M: f32 = 0.984;
const ARM_WIDTH_M: f32 = 0.125;
const ARM_HEIGHT_M: f32 = 0.225;
/// Diagnostic obstacle size shown in the same location as the sim obstacle.
const DIAGNOSTIC_OBSTACLE_RADIUS: f32 = 0.2;
const DIAGNOSTIC_OBSTACLE_HEIGHT: f32 = 0.8;

/// Diagnostic obstacle center in the canonical `map` frame
/// `environment_sim::startup::DIAGNOSTIC_OBSTACLE_POSITION_MAP`; converted to Bevy
/// coordinate frame at runtime below.
const DIAGNOSTIC_OBSTACLE_POSITION_MAP: Vec3 =
    Vec3::new(1.0, 1.0, DIAGNOSTIC_OBSTACLE_HEIGHT * 0.5);
/// Diagnostic obstacle color.
const DIAGNOSTIC_OBSTACLE_COLOR: Color = Color::srgb(1.0, 0.0, 0.0);
/// X-axis color for the datum marker (red = +X right in map frame).
const DATUM_X_COLOR: Color = Color::srgb(1.0, 0.0, 0.0);
/// Y-axis color for the datum marker (green = +Y forward in map frame).
const DATUM_Y_COLOR: Color = Color::srgb(0.0, 1.0, 0.0);
/// Z-axis color for the datum marker (blue = +Z up in map frame).
const DATUM_Z_COLOR: Color = Color::srgb(0.0, 0.0, 1.0);
/// Fixed camera location looking forward along map +Y (Bevy -Z).
const CAMERA_POSITION: Vec3 = Vec3::new(0.0, 10.0, 0.0);

/// Creates the ground plane and translucent boundary walls.
pub fn setup_scene(
    mut commands: Commands<'_, '_>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
    mut ambient_light: ResMut<'_, GlobalAmbientLight>,
    config: Res<'_, VisualizerConfig>,
) {
    let config = config.into_inner();
    ambient_light.color = Color::WHITE;
    ambient_light.brightness = AMBIENT_LIGHT_BRIGHTNESS;

    let [wx, _, wz] = config.world_size();
    let t = config.wall_thickness();
    let hy = WALL_RENDER_HEIGHT * 0.5;

    // The visual ground is a simple plane centered at the world origin.
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(wx, wz))),
        MeshMaterial3d(materials.add(GROUND_COLOR)),
        Transform::from_translation(Vec3::ZERO),
    ));

    let wall_mat = materials.add(StandardMaterial {
        base_color: WALL_COLOR,
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

    // Four thin cuboids outline the X/Z extents of the configured world.
    let walls = [
        (
            Vec3::new(t, WALL_RENDER_HEIGHT, wz),
            Vec3::new(-(wx + t) * 0.5, hy, 0.0),
        ),
        (
            Vec3::new(t, WALL_RENDER_HEIGHT, wz),
            Vec3::new((wx + t) * 0.5, hy, 0.0),
        ),
        (
            Vec3::new(wx, WALL_RENDER_HEIGHT, t),
            Vec3::new(0.0, hy, -(wz + t) * 0.5),
        ),
        (
            Vec3::new(wx, WALL_RENDER_HEIGHT, t),
            Vec3::new(0.0, hy, (wz + t) * 0.5),
        ),
    ];

    for (size, center) in walls {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(size.x, size.y, size.z))),
            MeshMaterial3d(wall_mat.clone()),
            Transform::from_translation(center),
        ));
    }
}

/// Draws the red diagnostic obstacle used for visual alignment with the sim.
pub fn setup_diagnostic_obstacle(
    mut commands: Commands<'_, '_>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
    frames: Res<'_, FrameConversion>,
) {
    let frames = frames.into_inner();
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(
            DIAGNOSTIC_OBSTACLE_RADIUS,
            DIAGNOSTIC_OBSTACLE_HEIGHT,
        ))),
        MeshMaterial3d(materials.add(DIAGNOSTIC_OBSTACLE_COLOR)),
        Transform::from_translation(frames.map_to_bevy(DIAGNOSTIC_OBSTACLE_POSITION_MAP)),
    ));
}

/// Creates a fixed perspective camera looking north along map +Y.
pub fn setup_camera(mut commands: Commands<'_, '_>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(CAMERA_POSITION).looking_at(Vec3::ZERO, Vec3::Z),
    ));
}

/// Creates the top-left HUD text entity.
pub fn setup_hud(mut commands: Commands<'_, '_>) {
    commands.spawn((
        HudText,
        Text::new(IDLE_HUD_TEXT),
        TextFont::from_font_size(HUD_FONT_SIZE),
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(HUD_LEFT_PX),
            top: Val::Px(HUD_TOP_PX),
            ..default()
        },
    ));
}

/// Creates the visual agent body at its configured initial position, with a
/// black arm marker on top toward the `-Z` front face (the declared
/// body-forward axis, where Spot's arm mounts) so a heading flip is visible
/// at a glance.
pub fn setup_visual_agent(
    mut commands: Commands<'_, '_>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
    config: Res<'_, VisualizerConfig>,
) {
    let config = config.into_inner();
    for agent_config in &config.app.agents {
        let arm_z = -(agent_config.size[0] - ARM_LENGTH_M) * 0.5;
        let arm_y = ARM_HEIGHT_M.mul_add(0.5, agent_config.size[2] * 0.5);

        commands
            .spawn((
                VisualAgent,
                crate::world::AgentId(agent_config.runtime.agent_id.clone()),
                Mesh3d(meshes.add(Cuboid::new(
                    agent_config.size[1],
                    agent_config.size[2],
                    agent_config.size[0],
                ))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: AGENT_COLOR,
                    alpha_mode: AlphaMode::Blend,
                    ..default()
                })),
                Transform::from_translation(Vec3::from_array(agent_config.initial_position)),
            ))
            .with_children(|parent| {
                parent.spawn((
                    Mesh3d(meshes.add(Cuboid::new(ARM_WIDTH_M, ARM_HEIGHT_M, ARM_LENGTH_M))),
                    MeshMaterial3d(materials.add(StandardMaterial {
                        base_color: ARM_MARKER_COLOR,
                        ..default()
                    })),
                    Transform::from_xyz(0.0, arm_y, arm_z),
                ));
            });
    }
}

/// Spawns one entity per catalog object that has a starting pose, recording
/// each in `VisualObjects` so the runtime pose poll can find it by
/// `object_id` without a scan. A real, MOCAP-tracked object (no
/// `virtual_initial_pose_map`) has no sane placeholder pose
/// to spawn at here — it can be placed anywhere on the MOCAP stage — so it
/// is spawned lazily instead, by `poll_object_state_system` on its first
/// observed `ObjectState`.
pub fn setup_visual_objects(
    mut commands: Commands<'_, '_>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
    catalog: Res<'_, ObjectCatalogRes>,
    frames: Res<'_, FrameConversion>,
    mut visual_objects: ResMut<'_, VisualObjects>,
) {
    let (catalog, frames) = (catalog.into_inner(), frames.into_inner());
    for object in catalog.0.iter() {
        let Some(pose) = object.virtual_initial_pose_map() else {
            continue;
        };
        let entity = spawn_visual_object(
            &mut commands,
            &mut meshes,
            &mut materials,
            frames,
            object,
            pose,
        );
        visual_objects
            .0
            .insert(object.object_id().to_owned(), entity);
    }
}

/// Spawns one catalog object's render entity at a given pose. Shared by
/// `setup_visual_objects` (catalog starting pose, at `Startup`) and
/// `poll_object_state_system` (first observed `ObjectState`, for an object
/// with no catalog starting pose). Color comes from the catalog's
/// descriptive color name; an unrecognized shape falls back to a sphere
/// placeholder rather than panicking.
pub fn spawn_visual_object(
    commands: &mut Commands<'_, '_>,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    frames: &FrameConversion,
    object: &core_msgs::catalog::ObjectSpec,
    pose: PoseXYZYaw,
) -> Entity {
    let (mesh, rotation) = match object.shape() {
        "cylinder" => (
            meshes.add(Cylinder::new(object.radius_m(), object.length_m())),
            object_rotation(frames, pose.yaw_rad),
        ),
        "box" => (
            meshes.add(Cuboid::new(
                object.width_m(),
                object.height_m(),
                object.length_m(),
            )),
            forward_facing_rotation(frames, pose.yaw_rad),
        ),
        _ => (
            meshes.add(Sphere::new(object.radius_m().max(0.02))),
            object_rotation(frames, pose.yaw_rad),
        ),
    };
    let translation = frames.map_to_bevy(Vec3::new(pose.x, pose.y, pose.z));

    let transform = Transform::from_translation(translation).with_rotation(rotation);
    commands
        .spawn((
            VisualObject,
            ObjectMotion::new(&transform),
            Mesh3d(mesh),
            MeshMaterial3d(materials.add(color_from_name(object.color()))),
            transform,
        ))
        .id()
}

/// Maps a catalog color name to a render color. Unknown names fall back to a
/// neutral gray rather than panicking.
fn color_from_name(name: &str) -> Color {
    match name {
        "pink" => Color::srgb(1.0, 0.41, 0.71),
        "red" => Color::srgb(0.9, 0.1, 0.1),
        "green" => Color::srgb(0.1, 0.8, 0.1),
        "blue" => Color::srgb(0.1, 0.3, 0.9),
        "yellow" => Color::srgb(0.95, 0.85, 0.1),
        _ => Color::srgb(0.6, 0.6, 0.6),
    }
}

/// Draws datum axes at the world origin in map terms.
///
/// Red = +X (right), Green = +Y (forward), Blue = +Z (up). Each arrow
/// direction is derived from a `map` unit vector through `FrameConversion`
/// rather than hardcoded as a Bevy direction, so the gizmo cannot silently
/// drift out of sync with the frame registration.
pub fn draw_datum_axes(
    mut commands: Commands<'_, '_>,
    mut gizmo_assets: ResMut<'_, Assets<GizmoAsset>>,
    frames: Res<'_, FrameConversion>,
) {
    let frames = frames.into_inner();
    let mut gizmo = GizmoAsset::default();
    let axis = |map_unit: Vec3| frames.map_to_bevy_vector(map_unit) * DATUM_AXIS_LENGTH;

    gizmo.arrow(Vec3::ZERO, axis(Vec3::X), DATUM_X_COLOR);
    gizmo.arrow(Vec3::ZERO, axis(Vec3::Y), DATUM_Y_COLOR);
    gizmo.arrow(Vec3::ZERO, axis(Vec3::Z), DATUM_Z_COLOR);

    commands.spawn(Gizmo {
        handle: gizmo_assets.add(gizmo),
        ..default()
    });
}
