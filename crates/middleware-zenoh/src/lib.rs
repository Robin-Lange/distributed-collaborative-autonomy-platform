//! Shared Zenoh/DDS bridge helpers.
//!
//! This module hides the repetitive middleware work: opening a session,
//! declaring publishers/subscribers, encoding messages, decoding received
//! samples, and draining crossbeam channels safely.

use core_msgs::{AgentScoped, key_expr_namespace};
use crossbeam_channel::{Receiver, Sender, TryRecvError, TrySendError};
use futures::executor::block_on;
use serde::{Serialize, de::DeserializeOwned};
use std::collections::HashMap;
use std::env;
use std::sync::{Mutex, OnceLock, PoisonError};
use zenoh::{
    liveliness::LivelinessToken,
    pubsub::{Publisher, Subscriber},
    qos::CongestionControl,
    sample::SampleKind,
};

pub use zenoh::Session;

const MULTICAST_INTERFACE_ENV: &str = "DECENTRALIZED_AUTONOMY_MULTICAST_IFACE";

#[derive(Clone, Debug, PartialEq, Eq)]
enum MulticastScouting {
    Enabled(String),
    Disabled,
}

fn encode_json<T: Serialize>(message: &T) -> Result<Vec<u8>, serde_json::Error> {
    serde_json::to_vec(message)
}

fn decode_json<T: DeserializeOwned>(payload: &[u8]) -> Result<T, serde_json::Error> {
    serde_json::from_slice(payload)
}

/// Bounded-channel capacity shared by every DDS bridge thread.
///
/// 1024 slots times the largest message on the bus (a few hundred bytes) caps
/// worst-case backlog memory at a few hundred KB per channel — bounded by
/// construction, and large enough that only a real stall (not steady-state
/// jitter) ever fills one.
pub const DDS_CHANNEL_CAPACITY: usize = 1024;

/// `QoS` policy used for publishers that should not silently drop messages.
///
/// `CongestionControl::Drop` ensures that a persistently full network queue
/// drops the sample rather than blocking the calling process's bus thread,
/// optimizing for ultra-low latency real-time commands.
const fn reliable_qos() -> CongestionControl {
    CongestionControl::Drop
}

/// Opens a Zenoh session for DDS-style message exchange.
///
/// The listen endpoint uses an ephemeral TCP port so multiple local processes
/// can run at the same time without hard-coding a port.
///
/// # Panics
///
/// Panics when Zenoh cannot open the configured session, including after the
/// multicast-disabled fallback attempt.
#[must_use]
pub fn open_session() -> Session {
    let multicast = multicast_scouting(env::var(MULTICAST_INTERFACE_ENV).ok().as_deref());
    let config = session_config(&multicast);

    match block_on(async { zenoh::open(config).await }) {
        Ok(session) => session,
        Err(first_error) if matches!(multicast, MulticastScouting::Enabled(_)) => {
            eprintln!(
                "dds_support: Zenoh multicast startup failed ({first_error}); retrying with multicast scouting disabled"
            );
            block_on(async { zenoh::open(session_config(&MulticastScouting::Disabled)).await })
                .unwrap_or_else(|fallback_error| {
                    panic!(
                        "failed to open zenoh session after disabling multicast scouting: {fallback_error}"
                    )
                })
        }
        Err(error) => panic!("failed to open zenoh session: {error}"),
    }
}

fn multicast_scouting(value: Option<&str>) -> MulticastScouting {
    match value.map(str::trim) {
        Some(value) if value.eq_ignore_ascii_case("disabled") => MulticastScouting::Disabled,
        Some("") | None => MulticastScouting::Enabled("auto".to_owned()),
        Some(value) => MulticastScouting::Enabled(value.to_owned()),
    }
}

fn session_config(multicast: &MulticastScouting) -> zenoh::Config {
    let mut config = zenoh::Config::default();

    config
        .insert_json5("mode", r#""peer""#)
        .unwrap_or_else(|error| panic!("invalid zenoh mode override: {error}"));

    config
        .insert_json5("listen/endpoints", r#"{peer: ["tcp/0.0.0.0:0"]}"#)
        .unwrap_or_else(|error| panic!("invalid zenoh listen-endpoint override: {error}"));

    match multicast {
        MulticastScouting::Enabled(interface) => {
            let interface = serde_json::to_string(interface)
                .expect("a Rust string always serializes as a JSON string");
            config
                .insert_json5("scouting/multicast/interface", &interface)
                .unwrap_or_else(|error| {
                    panic!("invalid zenoh scouting multicast interface override: {error}")
                });
        }
        MulticastScouting::Disabled => {
            config
                .insert_json5("scouting/multicast/enabled", "false")
                .unwrap_or_else(|error| {
                    panic!("invalid zenoh multicast disable override: {error}")
                });
        }
    }

    config
}

/// Builds the fully qualified key expression for a topic.
///
/// The namespace can be overridden with `DECENTRALIZED_AUTONOMY_DDS_NAMESPACE`
/// so multiple deployments can avoid sharing the same bus prefix.
fn topic_key_expr(topic: &str) -> String {
    let namespace = env::var("DECENTRALIZED_AUTONOMY_DDS_NAMESPACE")
        .unwrap_or_else(|_| key_expr_namespace().to_owned());

    format!("{namespace}/{topic}")
}

/// Declares a background subscriber for one shared topic.
///
/// # Panics
///
/// Panics when Zenoh cannot declare the subscriber.
pub fn subscribe_for_each<T>(
    session: &Session,
    topic: &'static str,
    on_message: impl Fn(T) + Send + Sync + 'static,
) -> Subscriber<()>
where
    T: prost::Message + Default + Clone + Send + 'static,
{
    block_on(async {
        session
            .declare_subscriber(topic_key_expr(topic))
            .callback(move |sample| {
                if let Ok(message) = T::decode(sample.payload().to_bytes().as_ref()) {
                    on_message(message);
                } else {
                    let count = decode_failures().note(topic);
                    log_rate_limited(count, || {
                        format!(
                            "dds_support: failed to decode message #{count} on topic \"{topic}\""
                        )
                    });
                }
            })
            .await
    })
    .unwrap_or_else(|error| panic!("failed to declare zenoh subscriber for \"{topic}\": {error}"))
}

/// Declares a background subscriber for one serde/JSON domain topic.
///
/// # Panics
///
/// Panics when Zenoh cannot declare the subscriber.
pub fn subscribe_json_for_each<T>(
    session: &Session,
    topic: &'static str,
    on_message: impl Fn(T) + Send + Sync + 'static,
) -> Subscriber<()>
where
    T: DeserializeOwned + Send + 'static,
{
    block_on(async {
        session
            .declare_subscriber(topic_key_expr(topic))
            .callback(move |sample| {
                if let Ok(message) = decode_json(sample.payload().to_bytes().as_ref()) {
                    on_message(message);
                } else {
                    let count = decode_failures().note(topic);
                    log_rate_limited(count, || {
                        format!(
                            "dds_support: failed to decode JSON message #{count} on topic \"{topic}\""
                        )
                    });
                }
            })
            .await
    })
    .unwrap_or_else(|error| panic!("failed to declare zenoh subscriber for \"{topic}\": {error}"))
}

/// Forwards serde/JSON domain samples into a bounded event channel.
#[must_use]
pub fn subscribe_json_forward<T>(
    session: &Session,
    topic: &'static str,
    tx: &Sender<T>,
) -> Subscriber<()>
where
    T: DeserializeOwned + Send + 'static,
{
    let tx = tx.clone();
    subscribe_json_for_each(session, topic, move |message| {
        send_or_log_drop(&tx, topic, message);
    })
}

/// Declares a reliable publisher for one shared topic.
///
/// # Panics
///
/// Panics when Zenoh cannot declare the publisher.
#[must_use]
pub fn declare_publisher(session: &Session, topic: &str) -> Publisher<'static> {
    block_on(async {
        session
            .declare_publisher(topic_key_expr(topic))
            .congestion_control(reliable_qos())
            .await
    })
    .unwrap_or_else(|error| panic!("failed to declare zenoh publisher for \"{topic}\": {error}"))
}

/// Subscribes to a topic's liveliness, invoking `on_change(alive)` on each event.
///
/// `history(true)` makes a subscriber declared after the token already exists
/// still receive the current state, so start order between token and subscriber
/// does not matter. `Put` means the token is alive, `Delete` means it was
/// withdrawn (process/session death).
///
/// # Panics
///
/// Panics when Zenoh cannot declare the liveliness subscriber.
pub fn subscribe_liveliness(
    session: &Session,
    topic: &str,
    on_change: impl Fn(bool) + Send + Sync + 'static,
) -> Subscriber<()> {
    block_on(async {
        session
            .liveliness()
            .declare_subscriber(topic_key_expr(topic))
            .callback(move |sample| on_change(sample.kind() == SampleKind::Put))
            .history(true)
            .await
    })
    .unwrap_or_else(|error| {
        panic!("failed to declare zenoh liveliness subscriber for \"{topic}\": {error}")
    })
}

/// Declares a liveliness token tied to the supplied session.
///
/// The caller must retain the returned token for as long as the advertised
/// adapter remains available.
///
/// # Panics
///
/// Panics when Zenoh cannot declare the liveliness token.
pub fn declare_liveliness_token(session: &Session, topic: &str) -> LivelinessToken {
    block_on(async {
        session
            .liveliness()
            .declare_token(topic_key_expr(topic))
            .await
    })
    .unwrap_or_else(|error| {
        panic!("failed to declare zenoh liveliness token for \"{topic}\": {error}")
    })
}

/// `QoS` policy used for safety-critical edge-triggered events where every sample must be delivered.
const fn safety_qos() -> CongestionControl {
    CongestionControl::Block
}

/// Declares a safety-critical publisher for one shared topic.
///
/// # Panics
///
/// Panics when Zenoh cannot declare the publisher.
#[must_use]
pub fn declare_safety_publisher(session: &Session, topic: &str) -> Publisher<'static> {
    block_on(async {
        session
            .declare_publisher(topic_key_expr(topic))
            .congestion_control(safety_qos())
            .priority(zenoh::qos::Priority::RealTime)
            .await
    })
    .unwrap_or_else(|error| {
        panic!("failed to declare safety zenoh publisher for \"{topic}\": {error}")
    })
}

/// Per-topic occurrence counters used to rate-limit failure logging.
///
/// Kept as a name-keyed map rather than one `static` per call site: topics are
/// stable strings and this crate has many call sites, so one small map is
/// simpler than threading a counter type through every caller for the same
/// effect.
struct FailureCounters(Mutex<HashMap<&'static str, u64>>);

impl FailureCounters {
    fn new() -> Self {
        Self(Mutex::new(HashMap::new()))
    }

    /// Records one more occurrence for `topic` and returns the new count.
    fn note(&self, topic: &'static str) -> u64 {
        // A poisoned map only means another thread panicked mid-update; the
        // counters themselves stay valid, so keep counting.
        let mut counts = self.0.lock().unwrap_or_else(PoisonError::into_inner);
        let count = counts.entry(topic).or_insert(0);
        *count += 1;
        let latest = *count;
        drop(counts);
        latest
    }
}

/// Log only the first occurrence and every 100th one after that, so a
/// persistent failure is visible without flooding stderr.
const LOG_EVERY_NTH: u64 = 100;

fn log_rate_limited(count: u64, message: impl FnOnce() -> String) {
    if count == 1 || count.is_multiple_of(LOG_EVERY_NTH) {
        eprintln!("{}", message());
    }
}

fn decode_failures() -> &'static FailureCounters {
    static COUNTERS: OnceLock<FailureCounters> = OnceLock::new();
    COUNTERS.get_or_init(FailureCounters::new)
}

fn publish_failures() -> &'static FailureCounters {
    static COUNTERS: OnceLock<FailureCounters> = OnceLock::new();
    COUNTERS.get_or_init(FailureCounters::new)
}

fn full_channel_drops() -> &'static FailureCounters {
    static COUNTERS: OnceLock<FailureCounters> = OnceLock::new();
    COUNTERS.get_or_init(FailureCounters::new)
}

/// Forwards all available protobuf samples into a local channel via background callback.
///
/// `from_proto` keeps protobuf conversion at the middleware boundary so callers
/// can work with internal Rust message types. Every sample is treated as an
/// independent event: if the channel is full the newest sample is dropped and
/// logged rather than evicting one nobody has seen yet.
pub fn subscribe_forward<T, P>(
    session: &Session,
    topic: &'static str,
    tx: &Sender<T>,
    from_proto: impl Fn(P) -> T + Send + Sync + 'static,
) -> Subscriber<()>
where
    P: prost::Message + Default + Clone + Send + 'static,
    T: Send + 'static,
{
    let tx_clone = tx.clone();
    subscribe_for_each(session, topic, move |message: P| {
        send_or_log_drop(&tx_clone, topic, from_proto(message));
    })
}

/// Forwards all available protobuf samples into a bounded latest-wins channel via callback.
///
/// Only appropriate for streams where a consumer only ever cares about the
/// newest sample (e.g. `AgentStateMsg`): on `Full`, `evict_rx` is used to pop
/// the oldest queued sample first so the consumer keeps seeing fresh data
/// instead of falling behind by a full channel's worth of stale samples.
pub fn subscribe_forward_latest<T, P>(
    session: &Session,
    topic: &'static str,
    tx: &Sender<T>,
    evict_rx: &Receiver<T>,
    from_proto: impl Fn(P) -> T + Send + Sync + 'static,
) -> Subscriber<()>
where
    P: prost::Message + Default + Clone + Send + 'static,
    T: Send + 'static,
{
    let tx_clone = tx.clone();
    let evict_rx_clone = evict_rx.clone();
    subscribe_for_each(session, topic, move |message: P| {
        send_drop_oldest(&tx_clone, &evict_rx_clone, from_proto(message));
    })
}

/// Publishes all messages currently queued in a local channel.
///
/// `to_proto` converts internal message types into protobuf messages only when
/// they cross the middleware boundary. A `put()` failure is counted under
/// `topic` and logged (first occurrence, then every 100th) instead of being
/// silently swallowed.
pub fn publish_available<T, P>(
    publisher: &Publisher<'_>,
    topic: &'static str,
    rx: &Receiver<T>,
    to_proto: impl Fn(T) -> P,
) where
    P: prost::Message,
{
    while let Ok(message) = rx.try_recv() {
        if block_on(async { publisher.put(to_proto(message).encode_to_vec()).await }).is_err() {
            let count = publish_failures().note(topic);
            log_rate_limited(count, || {
                format!("dds_support: publish failed #{count} on topic \"{topic}\"")
            });
        }
    }
}

/// Publishes all serde/JSON domain messages currently queued locally.
pub fn publish_json_available<T>(publisher: &Publisher<'_>, topic: &'static str, rx: &Receiver<T>)
where
    T: Serialize,
{
    while let Ok(message) = rx.try_recv() {
        let Ok(payload) = encode_json(&message) else {
            let count = publish_failures().note(topic);
            log_rate_limited(count, || {
                format!("dds_support: JSON encode failed #{count} on topic \"{topic}\"")
            });
            continue;
        };

        if block_on(async { publisher.put(payload).await }).is_err() {
            let count = publish_failures().note(topic);
            log_rate_limited(count, || {
                format!("dds_support: publish failed #{count} on topic \"{topic}\"")
            });
        }
    }
}

/// Sends an independent event into a bounded channel.
///
/// For commands, statuses, constraints, vetoes, and intents, on `Full` the new
/// message is dropped and logged as an error rather than evicting a
/// still-unseen event. Returns `false` only when the receiver has
/// disconnected.
pub fn send_or_log_drop<T>(tx: &Sender<T>, topic: &'static str, message: T) -> bool {
    match tx.try_send(message) {
        Ok(()) => true,
        Err(TrySendError::Disconnected(_)) => false,
        Err(TrySendError::Full(_)) => {
            let count = full_channel_drops().note(topic);
            log_rate_limited(count, || {
                format!("dds_support: channel full, dropped message #{count} on topic \"{topic}\"")
            });
            true
        }
    }
}

/// Sends into a bounded latest-wins channel.
///
/// On `Full`, this evicts the oldest queued message via `evict_rx` before
/// pushing the new one, so the consumer
/// always sees the newest sample instead of stalling behind a backlog. Sound
/// only when a single producer owns `tx` (true for every channel this is used
/// on) — concurrent producers could race on which message gets evicted.
pub fn send_drop_oldest<T>(tx: &Sender<T>, evict_rx: &Receiver<T>, message: T) -> bool {
    let mut pending = message;
    loop {
        match tx.try_send(pending) {
            Ok(()) => return true,
            Err(TrySendError::Disconnected(_)) => return false,
            Err(TrySendError::Full(returned)) => {
                pending = returned;
                let _ = evict_rx.try_recv();
            }
        }
    }
}

/// Result of draining a channel.
///
/// This carries both the newest matching message and whether the channel was
/// disconnected, so callers can distinguish "no data yet" from "producer gone."
#[derive(Clone, Debug)]
pub struct DrainStatus<T> {
    latest: Option<T>,
    disconnected: bool,
}

impl<T> DrainStatus<T> {
    /// Creates a status for a still-connected channel.
    const fn connected(latest: Option<T>) -> Self {
        Self {
            latest,
            disconnected: false,
        }
    }

    /// Creates a status for a disconnected channel.
    const fn disconnected(latest: Option<T>) -> Self {
        Self {
            latest,
            disconnected: true,
        }
    }

    /// Splits the status into `(latest_message, disconnected)`.
    pub fn into_parts(self) -> (Option<T>, bool) {
        (self.latest, self.disconnected)
    }

    /// Returns only the newest matching message.
    pub fn into_latest(self) -> Option<T> {
        self.latest
    }
}

/// Drains a channel and returns the newest message for one agent.
#[must_use]
pub fn drain_latest_for_agent<T: AgentScoped>(rx: &Receiver<T>, agent_id: &str) -> Option<T> {
    drain_latest_for_agent_with_status(rx, agent_id).into_latest()
}

/// Drains a channel for one agent and reports whether the channel disconnected.
#[must_use]
pub fn drain_latest_for_agent_with_status<T: AgentScoped>(
    rx: &Receiver<T>,
    agent_id: &str,
) -> DrainStatus<T> {
    drain_latest_with_status(rx, |message| message.agent_id() == agent_id)
}

/// Drains a channel, keeping the newest message per agent.
///
/// `try_recv` empties the whole channel in one pass, so a single-agent filter
/// would silently
/// discard every other agent's sample — this keeps one slot per `agent_id()`
/// instead of one slot total).
#[must_use]
pub fn drain_latest_per_agent_with_status<T: AgentScoped>(
    rx: &Receiver<T>,
) -> (HashMap<String, T>, bool) {
    let mut latest: HashMap<String, T> = HashMap::new();
    loop {
        match rx.try_recv() {
            Ok(message) => {
                latest.insert(message.agent_id().to_owned(), message);
            }
            Err(TryRecvError::Empty) => return (latest, false),
            Err(TryRecvError::Disconnected) => return (latest, true),
        }
    }
}

/// Drains a channel, delivering every matching message in arrival order.
///
/// Use this instead of [`drain_latest_with_status`] for streams where each
/// message is an independent event (e.g. constraint events from multiple
/// producers): keeping only the newest sample would silently drop interleaved
/// events from other sources.
pub fn drain_each_with_status<T>(
    rx: &Receiver<T>,
    keep: impl Fn(&T) -> bool,
    mut on_message: impl FnMut(T),
) -> bool {
    loop {
        match rx.try_recv() {
            Ok(message) if keep(&message) => on_message(message),
            Ok(_) => {}
            Err(TryRecvError::Empty) => return false,
            Err(TryRecvError::Disconnected) => return true,
        }
    }
}

/// Drains a channel and keeps only the newest message matching `keep`.
///
/// Older matching messages are intentionally overwritten because live control
/// loops usually care about the latest state/status, not stale samples.
pub fn drain_latest_with_status<T>(rx: &Receiver<T>, keep: impl Fn(&T) -> bool) -> DrainStatus<T> {
    let mut latest = None;

    loop {
        match rx.try_recv() {
            Ok(message) if keep(&message) => latest = Some(message),
            Ok(_) => {}
            Err(TryRecvError::Empty) => {
                return DrainStatus::connected(latest);
            }
            Err(TryRecvError::Disconnected) => {
                return DrainStatus::disconnected(latest);
            }
        }
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/dds_support.rs"]
mod dds_support_tests;
