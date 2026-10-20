use super::*;

fn args(values: &[&str]) -> Vec<OsString> {
    values.iter().map(OsString::from).collect()
}

#[test]
fn no_mode_opens_interactive_launcher_with_preseeded_defaults() {
    let Ok(Invocation::Interactive(options)) = Invocation::from_args(Vec::new()) else {
        panic!(
            "a no-mode invocation must now open the interactive launcher and produce an interactive invocation"
        );
    };
    // Pre-seeded Setup starting selection: default `sim` mode, no toggles.
    assert!(!options.visualizer);
    assert!(!options.mcap_recorder);
    assert!(
        options
            .sidecar_mocap_to_body_translation_m
            .iter()
            .all(|value| value.abs() < f64::EPSILON)
    );
}

#[test]
fn launcher_rejects_all_direct_arguments() {
    for arg_list in [
        vec!["-v"],
        vec!["-log"],
        vec!["sim"],
        vec!["real"],
        vec!["hybrid"],
        vec!["hybrid", "-v", "-log"],
        vec!["sim", "-l"],
        vec!["sim", "real"],
        vec!["sim", "--teleport"],
    ] {
        let Err(error) = Invocation::from_args(args(&arg_list)) else {
            panic!("direct arguments {arg_list:?} must be rejected");
        };
        assert!(error.contains("passing mode or flags directly is not supported"));
    }

    #[cfg(unix)]
    {
        use std::os::unix::ffi::OsStringExt;

        let Err(invalid) =
            Invocation::from_args(vec![OsString::from("sim"), OsString::from_vec(vec![0xff])])
        else {
            panic!("non-UTF-8 options must be rejected deterministically");
        };
        assert!(
            invalid.contains("passing mode or flags directly is not supported")
                || invalid.contains("valid UTF-8")
        );
    }
}

#[test]
fn public_help_describes_the_release_executable() {
    let help = usage();
    assert!(help.starts_with("usage: dca\n"));
    assert!(!help.contains("cargo run"));
}

#[test]
fn component_invocation_preserves_component_arguments() {
    let Ok(Invocation::Component(component, component_args)) = Invocation::from_args(args(&[
        "--component",
        "spot_sidecar",
        "--agent-id",
        "agent_02",
    ])) else {
        panic!("known component invocation should parse and produce a component invocation");
    };
    assert_eq!(component.as_str(), "spot_sidecar");
    assert_eq!(component_args, args(&["--agent-id", "agent_02"]));
}

#[test]
fn observers_are_not_treated_as_required_runtime_components() {
    assert!(Component::EnvironmentSim.is_required_runtime());
    assert!(Component::Agent.is_required_runtime());
    assert!(Component::ObjectTracker.is_required_runtime());
    assert!(Component::SpotSidecar.is_required_runtime());
    assert!(!Component::Visualizer.is_required_runtime());
    assert!(!Component::McapRecorder.is_required_runtime());
}

#[test]
fn spot_shutdown_graces_cover_power_off_and_nested_cleanup() {
    assert!(SPOT_RUNTIME_SHUTDOWN_GRACE > std::time::Duration::from_secs(20));
    assert!(Component::SpotSidecar.shutdown_grace() > SPOT_RUNTIME_SHUTDOWN_GRACE);
    assert_eq!(
        Component::EnvironmentSim.shutdown_grace(),
        CHILD_SHUTDOWN_GRACE
    );
}

#[test]
fn child_output_uses_structured_levels_before_stream_fallbacks() {
    let info = "\u{1b}[2m2026-07-28T15:47:06.258790Z\u{1b}[0m \
                \u{1b}[32m INFO\u{1b}[0m bevy_render::renderer: ready";
    let warning = "\u{1b}[2m2026-07-28T15:53:38.308499Z\u{1b}[0m \
                   \u{1b}[33m WARN\u{1b}[0m bevy_winit::state: skipped event";
    let error = "\u{1b}[2m2026-07-28T15:53:38.308499Z\u{1b}[0m \
                 \u{1b}[31m ERROR\u{1b}[0m bevy_render::renderer: failed";

    assert!(child_output_event_kind("stderr", info) == EventKind::Info);
    assert!(child_output_event_kind("stderr", warning) == EventKind::Warning);
    assert!(child_output_event_kind("stderr", error) == EventKind::Error);
    assert!(
        child_output_event_kind(
            "stderr",
            "2026-07-28T15:47:06.258790Z INFO bevy_render::renderer: ready",
        ) == EventKind::Info
    );
}

#[test]
fn child_output_keeps_unstructured_stream_failures_conservative() {
    assert!(
        child_output_event_kind("stderr", "ModuleNotFoundError: No module named 'bosdyn'")
            == EventKind::Error
    );
    assert!(
        child_output_event_kind("stderr", "[PYI-1:ERROR] Failed to execute script")
            == EventKind::Error
    );
    assert!(child_output_event_kind("stdout", "environment_sim: running") == EventKind::Info);
    assert!(
        child_output_event_kind("stdout", "FATAL ERROR: Spot runtime failed") == EventKind::Error
    );
}

#[test]
fn real_sidecar_owns_virtual_objects_but_hybrid_sidecar_does_not() {
    let mut config = crate::core_msgs::config::AppConfig::default();
    let agent_id = config.agents[0].runtime.agent_id.clone();

    config.agents[0].manifest.backend = crate::core_msgs::config::AgentBackend::Hardware;
    assert!(sidecar_owns_virtual_object_state(Some(&config), &agent_id));

    config.agents[0].manifest.backend = crate::core_msgs::config::AgentBackend::Hybrid;
    assert!(!sidecar_owns_virtual_object_state(Some(&config), &agent_id));
    assert!(!sidecar_owns_virtual_object_state(None, &agent_id));
}

#[test]
fn component_invocation_rejects_missing_and_unknown_names() {
    let Err(missing) = Invocation::from_args(args(&["--component"])) else {
        panic!("missing component name must be rejected");
    };
    assert!(missing.contains("missing component name"));

    let Err(unknown) = Invocation::from_args(args(&["--component", "not-a-component"])) else {
        panic!("unknown component name must be rejected");
    };
    assert!(unknown.contains("unknown component"));
}
