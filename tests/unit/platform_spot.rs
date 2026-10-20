use core_msgs::{CapabilityRequirement, SpeedPreset};

use super::*;

#[test]
fn profile_advertises_only_implemented_capabilities() {
    let capabilities = capabilities();

    assert!(capabilities.supports(CapabilityRequirement::Hold));
    assert!(capabilities.supports(CapabilityRequirement::PlanarNavigation));
    assert!(!capabilities.supports(CapabilityRequirement::SpatialNavigation));
    assert!(capabilities.supports(CapabilityRequirement::Pick));
    assert!(capabilities.supports(CapabilityRequirement::Place));

    let navigation = capabilities.navigation[0];
    assert!(
        navigation
            .speed_presets
            .selected(SpeedPreset::Slow)
            .max_linear_speed_m_s
            < navigation
                .speed_presets
                .selected(SpeedPreset::Medium)
                .max_linear_speed_m_s
    );
    assert!(
        navigation
            .speed_presets
            .selected(SpeedPreset::Medium)
            .max_linear_speed_m_s
            < navigation
                .speed_presets
                .selected(SpeedPreset::Fast)
                .max_linear_speed_m_s
    );
}
