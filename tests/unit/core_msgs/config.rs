use super::*;

#[test]
fn app_config_round_trip_preserves_per_agent_backend_and_speed() {
    let mut config = AppConfig::default();
    let mut hybrid = AgentConfig::default();
    hybrid.runtime.agent_id = "agent_02".to_owned();
    hybrid.manifest.agent_id = "agent_02".to_owned();
    hybrid.manifest.backend = AgentBackend::Hybrid;
    hybrid.manifest.max_speed = SpeedPreset::Fast;
    config.agents.push(hybrid);

    let json = serde_json::to_string(&config).expect("app config serializes");
    let decoded: AppConfig = serde_json::from_str(&json).expect("app config deserializes");

    assert_eq!(decoded.agents.len(), 2);
    assert_eq!(decoded.agents[1].runtime.agent_id, "agent_02");
    assert_eq!(decoded.agents[1].manifest.agent_id, "agent_02");
    assert_eq!(decoded.agents[1].manifest.backend, AgentBackend::Hybrid);
    assert_eq!(decoded.agents[1].manifest.max_speed, SpeedPreset::Fast);
}

#[test]
fn older_agent_json_defaults_to_medium_speed() {
    let json = serde_json::to_value(AgentManifestConfig::default())
        .expect("manifest serializes")
        .as_object()
        .expect("manifest is an object")
        .iter()
        .filter(|(key, _)| key.as_str() != "max_speed")
        .map(|(key, value)| (key.clone(), value.clone()))
        .collect::<serde_json::Map<_, _>>();

    let manifest: AgentManifestConfig =
        serde_json::from_value(json.into()).expect("manifest deserializes");

    assert_eq!(manifest.max_speed, SpeedPreset::Medium);
}
