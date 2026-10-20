//! In-memory record representation shared by the DDS reader and MCAP writer.
//!
//! `RecordedSample` is deliberately schema-aware: every payload carries the topic
//! and schema name that the writer uses to route it to a pre-registered MCAP
//! channel. This avoids guessing channel metadata at write time.

#[derive(Clone, Debug)]
pub struct RecordedSample {
    pub(super) topic: &'static str,
    pub(super) schema_name: &'static str,
    pub(super) log_time_ns: u64,
    pub(super) publish_time_ns: u64,
    pub(super) payload: Vec<u8>,
}

/// Encodes a prost message as a protobuf-backed recorded sample.
pub fn recorded_proto_sample<T: prost::Message>(
    topic: &'static str,
    schema_name: &'static str,
    message: &T,
    time_ns: u64,
) -> RecordedSample {
    RecordedSample {
        topic,
        schema_name,
        log_time_ns: time_ns,
        publish_time_ns: time_ns,
        payload: message.encode_to_vec(),
    }
}

/// Encodes a serializable value as a JSON-backed recorded sample.
pub fn recorded_json_sample<T: serde::Serialize>(
    topic: &'static str,
    schema_name: &'static str,
    message: &T,
    time_ns: u64,
) -> Result<RecordedSample, serde_json::Error> {
    Ok(RecordedSample {
        topic,
        schema_name,
        log_time_ns: time_ns,
        publish_time_ns: time_ns,
        payload: serde_json::to_vec(message)?,
    })
}
