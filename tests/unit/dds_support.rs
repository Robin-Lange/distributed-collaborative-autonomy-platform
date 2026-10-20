use super::*;
use core_msgs::{
    AgentStateMsg, BehaviorPrimitive, BehaviorRequest, BehaviorSource, PositionTarget,
};
use crossbeam_channel::{TryRecvError, bounded};

#[test]
fn event_send_preserves_queued_message_when_full_and_reports_disconnect() {
    let (tx, rx) = bounded(1);

    assert!(send_or_log_drop(&tx, "test/event_send", 1));
    assert!(send_or_log_drop(&tx, "test/event_send", 2));
    assert_eq!(rx.try_recv(), Ok(1));
    assert_eq!(rx.try_recv(), Err(TryRecvError::Empty));

    drop(rx);
    assert!(!send_or_log_drop(&tx, "test/event_send", 3));
}

#[test]
fn latest_wins_send_evicts_only_the_oldest_queued_value() {
    let (tx, rx) = bounded(2);
    tx.send(1).unwrap();
    tx.send(2).unwrap();

    assert!(send_drop_oldest(&tx, &rx, 3));
    assert_eq!(rx.try_recv(), Ok(2));
    assert_eq!(rx.try_recv(), Ok(3));
    assert_eq!(rx.try_recv(), Err(TryRecvError::Empty));
}

#[test]
fn latest_wins_send_reports_a_disconnected_receiver() {
    let (tx, rx) = bounded::<i32>(1);
    drop(rx);

    assert!(!send_drop_oldest(&tx, &bounded::<i32>(1).1, 1));
}

#[test]
fn latest_drain_keeps_only_the_newest_match_and_reports_connection_state() {
    let (tx, rx) = bounded(4);
    for value in 1..=4 {
        tx.send(value).unwrap();
    }

    let (latest, disconnected) = drain_latest_with_status(&rx, |value| value % 2 == 0).into_parts();
    assert_eq!(latest, Some(4));
    assert!(!disconnected);

    drop(tx);
    let (latest, disconnected) = drain_latest_with_status(&rx, |_| true).into_parts();
    assert_eq!(latest, None);
    assert!(disconnected);
}

#[test]
fn latest_drain_returns_a_final_value_when_sender_disconnected() {
    let (tx, rx) = bounded(2);
    tx.send(10).unwrap();
    tx.send(20).unwrap();
    drop(tx);

    let (latest, disconnected) = drain_latest_with_status(&rx, |_| true).into_parts();
    assert_eq!(latest, Some(20));
    assert!(disconnected);
}

#[test]
fn event_drain_preserves_matching_arrival_order_and_reports_disconnect() {
    let (tx, rx) = bounded(5);
    for value in 1..=5 {
        tx.send(value).unwrap();
    }
    drop(tx);

    let mut received = Vec::new();
    let disconnected =
        drain_each_with_status(&rx, |value| value % 2 == 1, |value| received.push(value));

    assert_eq!(received, vec![1, 3, 5]);
    assert!(disconnected);
}

#[test]
fn agent_drain_filters_foreign_messages_and_keeps_latest_matching_state() {
    let (tx, rx) = bounded(4);
    for (agent_id, stamp_ms) in [("spot-2", 1), ("spot-1", 2), ("spot-1", 3), ("spot-2", 4)] {
        tx.send(AgentStateMsg {
            agent_id: agent_id.to_owned(),
            stamp_ms,
            ..Default::default()
        })
        .unwrap();
    }

    let latest = drain_latest_for_agent(&rx, "spot-1").unwrap();
    assert_eq!(latest.agent_id, "spot-1");
    assert_eq!(latest.stamp_ms, 3);
    assert!(matches!(rx.try_recv(), Err(TryRecvError::Empty)));
}

#[test]
fn failure_counters_are_independent_by_topic() {
    let counters = FailureCounters::new();

    assert_eq!(counters.note("topic-a"), 1);
    assert_eq!(counters.note("topic-a"), 2);
    assert_eq!(counters.note("topic-b"), 1);
}

#[test]
fn multicast_interface_defaults_to_auto_and_accepts_explicit_overrides() {
    assert_eq!(
        multicast_scouting(None),
        MulticastScouting::Enabled("auto".to_owned())
    );
    assert_eq!(
        multicast_scouting(Some("en0")),
        MulticastScouting::Enabled("en0".to_owned())
    );
    assert_eq!(
        multicast_scouting(Some(" disabled ")),
        MulticastScouting::Disabled
    );
}

#[test]
fn typed_domain_messages_round_trip_through_the_json_wire_codec() {
    let request = BehaviorRequest {
        request_id: "request-01".to_owned(),
        agent_id: "agent-01".to_owned(),
        source: BehaviorSource::Operator,
        behavior: BehaviorPrimitive::NavigateTo {
            target: PositionTarget::Planar([1.0, 2.0]),
            heading_yaw_rad: Some(0.5),
        },
        frame_id: "map".to_owned(),
        stamp_ms: 42,
    };

    let payload = encode_json(&request).expect("request serializes");
    let decoded: BehaviorRequest = decode_json(&payload).expect("request deserializes");

    assert_eq!(decoded, request);
}
