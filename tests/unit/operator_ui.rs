use super::*;
use crate::core_msgs::PoseXYZYaw;
use crate::core_msgs::{
    CapabilitySet, NavigationCapability, NavigationKind, SpeedLimits, SpeedPresets,
    command_type_velocity, veto_reason_unreachable,
};

#[test]
fn spot_runtime_progress_is_recognized_before_process_output_is_filtered() {
    let event = RuntimeEvent::new(
        EventKind::Info,
        "spot_sidecar:stdout",
        "spot-runtime: using checked-in Python sidecar",
    );

    assert_eq!(
        spot_runtime_status(&event),
        Some("using checked-in Python sidecar")
    );
    assert!(process_output_event(&event));
}

#[test]
fn unrelated_sidecar_output_is_not_a_runtime_status() {
    let event = RuntimeEvent::new(
        EventKind::Error,
        "spot_sidecar:stderr",
        "ModuleNotFoundError: No module named 'bosdyn'",
    );

    assert_eq!(spot_runtime_status(&event), None);
}

#[test]
fn sidecar_fatal_error_outranks_pyinstaller_footer() {
    let fatal = RuntimeEvent::new(
        EventKind::Info,
        "spot_sidecar:stdout",
        "FATAL ERROR: No module named 'bosdyn.client.resources'",
    );
    let footer = RuntimeEvent::new(
        EventKind::Error,
        "spot_sidecar:stderr",
        "[PYI-1:ERROR] Failed to execute script 'spot_sidecar'",
    );

    let Some((fatal_priority, fatal_detail)) = spot_sidecar_failure_detail(&fatal) else {
        panic!("fatal sidecar output must be recognized");
    };
    let Some((footer_priority, _)) = spot_sidecar_failure_detail(&footer) else {
        panic!("PyInstaller footer must be recognized as a fallback");
    };

    assert!(fatal_priority > footer_priority);
    assert_eq!(fatal_detail, "No module named 'bosdyn.client.resources'");
}

#[test]
fn lease_conflict_has_safe_operator_guidance() {
    let event = RuntimeEvent::new(
        EventKind::Info,
        "spot_sidecar:stdout",
        "FATAL ERROR: bosdyn.api.AcquireLeaseResponse \
         (ResourceAlreadyClaimedError): Use TakeLease method",
    );

    let Some((priority, detail)) = spot_sidecar_failure_detail(&event) else {
        panic!("lease conflict must be recognized");
    };

    assert_eq!(priority, 3);
    assert_eq!(detail, SPOT_LEASE_CONFLICT_GUIDANCE);
}

#[test]
fn keepalive_power_block_has_safe_operator_guidance() {
    let event = RuntimeEvent::new(
        EventKind::Info,
        "spot_sidecar:stdout",
        "FATAL ERROR: bosdyn.api.PowerCommandResponse \
         (KeepaliveMotorsOffError): Cannot power on",
    );

    let Some((priority, detail)) = spot_sidecar_failure_detail(&event) else {
        panic!("Keepalive power block must be recognized");
    };

    assert_eq!(priority, 3);
    assert_eq!(detail, SPOT_KEEPALIVE_MOTORS_OFF_GUIDANCE);
}

#[test]
fn process_failures_are_kept_in_the_operator_activity_log() {
    let fatal = RuntimeEvent::new(
        EventKind::Info,
        "spot_sidecar:stdout",
        "FATAL ERROR: No module named 'bosdyn.client.resources'",
    );
    let traceback = RuntimeEvent::new(
        EventKind::Error,
        "spot_sidecar:stderr",
        "ModuleNotFoundError: No module named 'bosdyn.client.resources'",
    );
    let routine_output = RuntimeEvent::new(
        EventKind::Info,
        "environment_sim:stdout",
        "environment_sim: running",
    );
    let process_warning = RuntimeEvent::new(
        EventKind::Warning,
        "visualizer:stderr",
        "WARN bevy_winit::state: skipped event",
    );

    assert!(activity_event_should_be_recorded(&fatal));
    assert!(activity_event_should_be_recorded(&traceback));
    assert!(activity_event_should_be_recorded(&process_warning));
    assert!(!activity_event_should_be_recorded(&routine_output));
}

#[test]
fn operator_activity_log_is_visible_before_shutdown() {
    let directory =
        std::env::temp_dir().join(format!("dca-operator-log-test-{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();
    let path = directory.join("operator_ui_test.md");
    let mut log = OperatorActivityLog::create(&path).unwrap();

    log.record(&RuntimeEvent::new(
        EventKind::Error,
        "test",
        "runtime failed",
    ))
    .unwrap();
    log.record(&RuntimeEvent::new(
        EventKind::Warning,
        "test",
        "runtime degraded",
    ))
    .unwrap();
    log.record(&RuntimeEvent::new(
        EventKind::Component,
        "test",
        "runtime started",
    ))
    .unwrap();

    let active_content = std::fs::read_to_string(&path).unwrap();
    assert!(active_content.contains("[ERROR] runtime failed"));
    assert!(active_content.contains("[WARN] runtime degraded"));
    assert!(active_content.contains("[INFO] runtime started"));
    assert!(!active_content.ends_with("```\n"));

    log.finish().unwrap();
    let finished_content = std::fs::read_to_string(&path).unwrap();
    assert!(finished_content.ends_with("```\n"));
    std::fs::remove_dir_all(directory).unwrap();
}

#[test]
fn mocap_bridge_exit_preserves_the_last_diagnostic() {
    assert_eq!(
        mocap_bridge_failure_detail(
            "mocap bridge exited with code 1",
            "mocap_zenoh_bridge: Address already in use (os error 48)"
        ),
        "mocap bridge exited with code 1: mocap_zenoh_bridge: Address already in use (os error 48)"
    );
    assert_eq!(
        mocap_bridge_failure_detail(
            "mocap bridge exited with code 1",
            "waiting for mocap bridge output..."
        ),
        "mocap bridge exited with code 1"
    );
}

#[test]
fn setup_field_cursor_stops_at_the_last_visible_field() {
    assert_eq!(next_setup_field_cursor(0, 4), 1);
    assert_eq!(next_setup_field_cursor(2, 3), 2);
    assert_eq!(next_setup_field_cursor(0, 1), 0);
    assert_eq!(next_setup_field_cursor(15, 0), 0);
}

#[test]
fn unspecified_manipulation_failure_directs_operator_to_runtime_log() {
    let mut status = ManipulationStatus {
        object_id: "block_mocap_01".to_owned(),
        phase: manipulation_phase_failed(),
        failure_reason: manipulation_failure_reason_unspecified(),
        ..Default::default()
    };

    let message = manipulation_status_message(&status);

    assert!(message.contains("platform execution failure"));
    assert!(message.contains("platform runtime log"));
    assert!(!message.contains("unspecified"));

    status.failure_reason = u8::MAX;
    let unknown_message = manipulation_status_message(&status);
    assert!(unknown_message.contains("platform execution failure"));
    assert!(unknown_message.contains("platform runtime log"));
    assert!(!unknown_message.contains("unknown"));
}

#[test]
fn manipulation_failures_explain_no_contact_and_unsafe_height() {
    let mut status = ManipulationStatus {
        object_id: "block_mocap_01".to_owned(),
        phase: manipulation_phase_failed(),
        failure_reason: manipulation_failure_reason_no_object_detected(),
        ..Default::default()
    };

    assert_eq!(
        manipulation_status_message(&status),
        "Manipulation failed: block_mocap_01. Reason: gripper reached fully closed without \
         detecting object contact."
    );

    status.failure_reason = manipulation_failure_reason_unsafe_grip_height();
    assert_eq!(
        manipulation_status_message(&status),
        "Manipulation failed: block_mocap_01. Reason: requested grip point is below the \
         platform's safe floor clearance."
    );
}

#[test]
fn arm_sector_veto_preserves_prose_and_object_ids() {
    let veto = CommandVeto {
        reason: veto_reason_unreachable(),
        detail: "block_mocap_01 is outside the arm's forward sector: 134.6° from the platform's \
                 heading; allowed offset is ±90.0°."
            .to_owned(),
        ..Default::default()
    };

    assert_eq!(
        veto_message(&veto),
        "Command vetoed: unreachable. block_mocap_01 is outside the arm's forward sector: \
         134.6° from the platform's heading; allowed offset is ±90.0°."
    );
}

fn tracked_state(stamp_ms: u64, pose_valid: bool) -> ObjectState {
    ObjectState {
        object_id: "test-object".to_owned(),
        lifecycle: object_lifecycle_free(),
        holder_agent_id: String::new(),
        pose: PoseXYZYaw {
            x: 1.25,
            y: -2.5,
            z: 0.4,
            yaw_rad: 0.0,
        },
        pose_valid,
        stamp_ms,
        orientation_map: None,
    }
}

#[test]
fn object_tracker_first_valid_pose_uses_object_state_timestamp() {
    let stamp_ms = 1_700_000_123_456;
    let events = object_tracker_events(None, &tracked_state(stamp_ms, true));

    assert_eq!(events.len(), 1);
    assert_eq!(
        events[0].created_wall_time.timestamp_millis(),
        stamp_ms.cast_signed()
    );
    assert!(events[0].message.contains("tracking at 1.25 -2.50 0.40"));
    assert_eq!(events[0].source, "object_tracker");
}

#[test]
fn object_tracker_repeated_valid_pose_does_not_flood_event_log() {
    let previous = tracked_state(100, true);
    let mut current = tracked_state(101, true);
    current.pose.x = 9.0;

    assert!(object_tracker_events(Some(&previous), &current).is_empty());
}

#[test]
fn object_tracker_pose_lost_and_reacquired_are_timestamped_events() {
    let valid = tracked_state(100, true);
    let lost = tracked_state(200, false);
    let reacquired = tracked_state(300, true);

    let lost_events = object_tracker_events(Some(&valid), &lost);
    assert_eq!(lost_events.len(), 1);
    assert!(lost_events[0].kind == EventKind::Warning);
    assert!(lost_events[0].message.contains("no valid pose"));
    assert_eq!(lost_events[0].created_wall_time.timestamp_millis(), 200);

    let reacquired_events = object_tracker_events(Some(&lost), &reacquired);
    assert_eq!(reacquired_events.len(), 1);
    assert!(reacquired_events[0].kind == EventKind::Info);
    assert!(reacquired_events[0].message.contains("tracking reacquired"));
    assert_eq!(
        reacquired_events[0].created_wall_time.timestamp_millis(),
        300
    );
}

#[test]
fn object_tracker_lifecycle_change_reports_holder_with_timestamp() {
    let free = tracked_state(100, true);
    let mut held = tracked_state(250, true);
    held.lifecycle = object_lifecycle_held();
    held.holder_agent_id = "agent_02".to_owned();

    let events = object_tracker_events(Some(&free), &held);

    assert_eq!(events.len(), 1);
    assert!(events[0].kind == EventKind::Command);
    assert!(events[0].message.contains("picked up by Agent 02"));
    assert_eq!(events[0].created_wall_time.timestamp_millis(), 250);
}

#[test]
fn object_tracker_holder_change_is_reported_without_a_lifecycle_transition() {
    let mut previous = tracked_state(100, true);
    previous.lifecycle = object_lifecycle_held();
    previous.holder_agent_id = "agent_01".to_owned();
    let mut current = previous.clone();
    current.stamp_ms = 150;
    current.holder_agent_id = "agent_03".to_owned();

    let events = object_tracker_events(Some(&previous), &current);

    assert_eq!(events.len(), 1);
    assert!(events[0].message.contains("holder changed to Agent 03"));
    assert_eq!(events[0].created_wall_time.timestamp_millis(), 150);
}

#[test]
fn platform_command_messages_use_canonical_x_y_order() {
    let mut command = PlatformCommand {
        command_type: command_type_velocity(),
        target_position: [1.25, -2.5, 0.0],
        ..Default::default()
    };

    assert_eq!(
        platform_command_event(&command, false).unwrap().1,
        "Driving to 1.25 -2.50."
    );

    command.command_type = command_type_hold();
    assert_eq!(
        platform_command_event(&command, true).unwrap().1,
        "Holding position at 1.25 -2.50."
    );

    command.command_type = command_type_stop();
    assert_eq!(
        platform_command_event(&command, false).unwrap().1,
        "Safety stop: robot stopped near 1.25 -2.50."
    );
}

#[test]
fn event_log_scrollbar_click_maps_top_and_bottom() {
    let scrollbar = event_log_scrollbar(Rect::new(4, 10, 20, 11), 30, 6);
    let max_scroll = runtime_event_max_scroll_for_body(30, 6);

    assert_eq!(
        event_log_scroll_from_scrollbar_row(scrollbar, 10),
        max_scroll
    );
    assert_eq!(event_log_scroll_from_scrollbar_row(scrollbar, 20), 0);
    assert_eq!(event_log_scroll_from_scrollbar_row(scrollbar, 15), 13);
}

#[test]
fn event_log_scrollbar_handles_empty_and_single_row_tracks() {
    let no_overflow = event_log_scrollbar(Rect::new(0, 0, 5, 10), 4, 6);
    assert_eq!(event_log_scroll_from_scrollbar_row(no_overflow, 7), 0);

    let single_row = event_log_scrollbar(Rect::new(0, 8, 1, 1), 100, 5);
    assert_eq!(event_log_scroll_from_scrollbar_row(single_row, 8), 0);
}

#[test]
fn wrapping_respects_width_and_unicode_character_boundaries() {
    assert_eq!(wrap_text("alpha beta gamma", 10), ["alpha beta", "gamma"]);
    assert_eq!(wrap_text("éééé", 2), ["éé", "éé"]);
    assert_eq!(wrap_text("", 8), [""]);
    assert!(wrap_text("anything", 0).is_empty());
}

#[test]
fn rectangle_hit_testing_uses_half_open_bounds() {
    let area = Rect::new(3, 5, 4, 2);

    assert!(rect_contains(area, 3, 5));
    assert!(rect_contains(area, 6, 6));
    assert!(!rect_contains(area, 7, 6));
    assert!(!rect_contains(area, 6, 7));
    assert!(!rect_contains(area, 2, 5));
}

#[test]
fn event_buffer_keeps_newest_first_and_excludes_process_output() {
    let mut events = VecDeque::new();
    push_recent_event(
        &mut events,
        RuntimeEvent::new(EventKind::Info, "onboard_agent", "first"),
    );
    push_recent_event(
        &mut events,
        RuntimeEvent::new(EventKind::Info, "environment_sim", "second"),
    );
    push_recent_event(
        &mut events,
        RuntimeEvent::new(
            EventKind::Error,
            "environment_sim:stderr",
            "raw child output",
        ),
    );

    assert_eq!(events.len(), 2);
    let Some(front) = events.front() else {
        panic!("newest event");
    };
    assert_eq!(front.message, "second");
    let Some(back) = events.back() else {
        panic!("oldest event");
    };
    assert_eq!(back.message, "first");
}

#[test]
fn connect_phase_resolves_timeout_and_success_correctly() {
    let start = Instant::now();

    assert_eq!(
        connect_phase(start, false, false, start),
        ConnectPhase::Waiting
    );
    assert_eq!(
        connect_phase(start, true, false, start),
        ConnectPhase::Waiting
    );
    assert_eq!(
        connect_phase(start, false, true, start),
        ConnectPhase::Waiting
    );
    assert_eq!(
        connect_phase(start, true, true, start),
        ConnectPhase::Connected
    );

    let later = start + CONNECT_TIMEOUT;
    assert_eq!(
        connect_phase(start, false, false, later),
        ConnectPhase::Failed
    );
    assert_eq!(
        connect_phase(start, true, false, later),
        ConnectPhase::Failed
    );
    assert_eq!(
        connect_phase(start, true, true, later),
        ConnectPhase::Connected
    );
}

#[test]
fn capability_disabled_reason_distinguishes_unknown_unsupported_and_offline() {
    assert_eq!(
        capability_disabled_reason(None, CapabilityRequirement::Hold),
        Some("awaiting capabilities".to_owned())
    );

    let mut report = CapabilityReport {
        agent_id: "agent-01".to_owned(),
        platform_profile: "test".to_owned(),
        connected: false,
        unavailable_reason: Some("adapter disconnected".to_owned()),
        capabilities: CapabilitySet {
            hold: true,
            navigation: vec![NavigationCapability {
                kind: NavigationKind::Planar,
                speed_presets: SpeedPresets {
                    slow: SpeedLimits {
                        max_linear_speed_m_s: 0.5,
                        max_yaw_rate_rad_s: 0.5,
                    },
                    medium: SpeedLimits {
                        max_linear_speed_m_s: 1.0,
                        max_yaw_rate_rad_s: 1.0,
                    },
                    fast: SpeedLimits {
                        max_linear_speed_m_s: 1.5,
                        max_yaw_rate_rad_s: 1.5,
                    },
                },
                position_tolerance_m: 0.1,
                supports_target_heading: false,
            }],
            manipulation: None,
        },
        stamp_ms: 42,
    };

    assert_eq!(
        capability_disabled_reason(Some(&report), CapabilityRequirement::Pick),
        Some("test does not support Pick".to_owned())
    );
    assert_eq!(
        capability_disabled_reason(Some(&report), CapabilityRequirement::Hold),
        Some("adapter disconnected".to_owned())
    );

    report.connected = true;
    assert_eq!(
        capability_disabled_reason(Some(&report), CapabilityRequirement::Hold),
        None
    );
}

#[test]
fn speed_label_uses_the_selected_platform_profile_limits() {
    let mut agent = crate::core_msgs::config::AgentConfig::default();
    agent.manifest.platform = platform_spot::PROFILE.to_owned();
    agent.manifest.max_speed = crate::core_msgs::SpeedPreset::Slow;

    assert_eq!(agent_speed_label(&agent), "slow (0.40 m/s, 0.50 rad/s)");
}
