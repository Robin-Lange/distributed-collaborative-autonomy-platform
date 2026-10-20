//! First-party platform profile lookup used by the local stack launcher.

use core_msgs::CapabilitySet;

/// Returns the static capabilities for a configured platform profile.
///
/// Adding a first-party platform requires adding its crate and one explicit
/// match arm here. Unknown names remain errors instead of inheriting a
/// universal capability set.
pub fn capabilities_for(profile: &str) -> Option<CapabilitySet> {
    match profile {
        platform_spot::PROFILE => Some(platform_spot::capabilities()),
        _ => None,
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/platform_profiles.rs"]
mod tests;
