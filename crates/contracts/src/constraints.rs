//! Source-owned active-constraint tracking.
//!
//! Consumers of `constraint_event` must not treat the stream as a single
//! last-writer-wins latch: independent producers (environment simulator, Spot
//! sidecar) raise and clear independent conditions. This module tracks the
//! active set keyed by `(source, constraint_type, object_id)` so a clear from
//! one producer can never erase another producer's active hard stop.
//!
//! Lifecycle rules (see the `ConstraintEvent` contract in `core_msgs.proto`):
//! - `severity_none` removes only the matching key.
//! - Environment-sourced entries are level-published (re-asserted every physics
//!   tick) and additionally expire when not re-asserted within
//!   `ENV_CONSTRAINT_TTL_MS` — a transport backstop, not the primary clear.
//! - Spot-adapter entries never expire; only the sidecar's matching clear
//!   releases them.
//! - `GOAL_BLOCKED` entries are one-shot and are dropped when a newer intent
//!   arrives (see `ActiveConstraints::drop_goal_blocked_older_than`).
//!
//! Shared by the onboard agent runtime and the operator UI so both derive the
//! same active set from the same event stream.

use crate::{
    ConstraintEvent, constraint_type_goal_blocked, constraint_type_name,
    constraint_type_robot_fault, constraint_type_virtual_wall, constraint_type_wall,
    severity_hard_stop, severity_none, source_environment_runtime, source_name,
};
use std::collections::BTreeMap;

/// How long an environment-sourced constraint stays active without re-assertion.
///
/// Walls re-assert every ~15.6 ms fixed tick while the condition
/// holds; 250 ms tolerates scheduling hiccups while still bounding how long a
/// lost clear event can pin the agent.
pub const ENV_CONSTRAINT_TTL_MS: u64 = 250;

/// Identity of one independently raised constraint condition.
type ConstraintKey = (u8, u8, String);

/// One active entry: the latest raising event plus local receipt time.
#[derive(Clone, Debug)]
struct ActiveEntry {
    event: ConstraintEvent,
    received_ms: u64,
}

/// What applying one event did to the active set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApplyOutcome {
    /// A new condition became active.
    Activated,

    /// An already-active condition was re-asserted.
    Refreshed,

    /// An active condition was cleared by its matching `severity_none`.
    Cleared,

    /// The event did not change the set (e.g. a clear for an unknown key).
    Ignored,
}

/// Active constraint conditions keyed by `(source, constraint_type, object_id)`.
///
/// Overseer mental model: this tracks the current safety stops, not a raw event
/// log. Every incoming event either refreshes one condition, clears its exact
/// matching condition, or is ignored.
#[derive(Default)]
pub struct ActiveConstraints {
    entries: BTreeMap<ConstraintKey, ActiveEntry>,
}

impl ActiveConstraints {
    /// Folds one received event into the active set.
    ///
    /// `now_ms` is the local receipt time, used for TTL expiry (producer stamps
    /// may come from another host's clock).
    pub fn apply(&mut self, event: ConstraintEvent, now_ms: u64) -> ApplyOutcome {
        let key = key_of(&event);

        if event.severity == severity_none() {
            return if self.entries.remove(&key).is_some() {
                ApplyOutcome::Cleared
            } else {
                ApplyOutcome::Ignored
            };
        }

        if event.severity != severity_hard_stop() {
            return ApplyOutcome::Ignored;
        }

        let outcome = if self.entries.contains_key(&key) {
            ApplyOutcome::Refreshed
        } else {
            ApplyOutcome::Activated
        };

        self.entries.insert(
            key,
            ActiveEntry {
                event,
                received_ms: now_ms,
            },
        );

        outcome
    }

    /// Expires environment-sourced entries that stopped being re-asserted.
    ///
    /// Spot-adapter entries are exempt: their lifecycle is owned by the sidecar
    /// and released only by an explicit matching clear.
    pub fn expire(&mut self, now_ms: u64) -> Vec<ConstraintEvent> {
        let mut expired = Vec::new();

        self.entries.retain(|_, entry| {
            let from_environment = entry.event.source == source_environment_runtime();
            let stale = now_ms.saturating_sub(entry.received_ms) > ENV_CONSTRAINT_TTL_MS;

            if from_environment && stale {
                expired.push(entry.event.clone());
                false
            } else {
                true
            }
        });

        expired
    }

    /// Drops goal-blocked reports that predate the newest intent.
    ///
    /// A blocked goal is a statement about a *previous* command; a fresh
    /// operator intent supersedes it. No other constraint type may be cleared
    /// by intent recency.
    pub fn drop_goal_blocked_older_than(&mut self, intent_stamp_ms: u64) {
        self.entries.retain(|_, entry| {
            entry.event.constraint_type != constraint_type_goal_blocked()
                || entry.event.stamp_ms >= intent_stamp_ms
        });
    }
    ///
    /// Only entries whose source passes `trusted` are considered (collision
    /// authority differs by backend). Robot faults outrank every other
    /// type; ties resolve to the newest stamp so diagnostics point at the most
    /// recent evidence.
    pub fn active_hard_stop(&self, trusted: impl Fn(u8) -> bool) -> Option<&ConstraintEvent> {
        self.entries
            .values()
            .map(|entry| &entry.event)
            .filter(|event| trusted(event.source))
            .max_by_key(|event| {
                let fault_priority =
                    u8::from(event.constraint_type == constraint_type_robot_fault());
                (fault_priority, event.stamp_ms)
            })
    }
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Returns whether no constraints are currently active.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Returns whether Avian currently reports a wall stop.
    ///
    /// These stops are command-dependent: after the agent sends STOP, Avian
    /// reclassifies the existing contact against the next target. A navigation
    /// request received before the matching clear must wait for that
    /// reclassification instead of being consumed by the previous command's
    /// collision episode.
    #[must_use]
    pub fn has_environment_wall_stop(&self) -> bool {
        self.entries.values().any(|entry| {
            entry.event.source == source_environment_runtime()
                && (entry.event.constraint_type == constraint_type_wall()
                    || entry.event.constraint_type == constraint_type_virtual_wall())
        })
    }

    /// One-line listing of every active condition, for vetoes and diagnostics.
    #[must_use]
    pub fn summary(&self) -> String {
        self.entries
            .values()
            .map(|entry| {
                let event = &entry.event;
                let object = if event.object_id.is_empty() {
                    "-"
                } else {
                    &event.object_id
                };
                format!(
                    "{}/{}[{}]",
                    source_name(event.source),
                    constraint_type_name(event.constraint_type),
                    object
                )
            })
            .collect::<Vec<_>>()
            .join(", ")
    }
}

/// Builds the set key for one event.
fn key_of(event: &ConstraintEvent) -> ConstraintKey {
    (event.source, event.constraint_type, event.object_id.clone())
}

#[cfg(test)]
#[path = "../../../tests/unit/core_msgs/constraints.rs"]
mod constraints_tests;
