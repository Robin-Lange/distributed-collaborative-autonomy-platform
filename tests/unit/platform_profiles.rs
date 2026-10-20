use core_msgs::CapabilityRequirement;

use super::*;

#[test]
fn known_profile_resolves_to_its_capabilities() {
    let capabilities = capabilities_for(platform_spot::PROFILE).expect("Spot is registered");

    assert!(capabilities.supports(CapabilityRequirement::PlanarNavigation));
}

#[test]
fn unknown_profile_does_not_gain_default_capabilities() {
    assert!(capabilities_for("unknown").is_none());
}
