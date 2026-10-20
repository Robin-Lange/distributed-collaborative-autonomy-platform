//! MCAP writer for recorded bus samples.
//!
//! The writer registers every expected topic/schema before writing samples, then
//! validates each incoming sample against that registry. That makes corrupted or
//! misrouted samples fail loudly instead of producing an ambiguous recording.

use super::{frame_transform, record::RecordedSample};
use core_msgs::{
    topic_agent_state, topic_authority_state, topic_command_status, topic_command_veto,
    topic_constraint_event, topic_frame_calibration, topic_manipulation_command,
    topic_manipulation_intent, topic_manipulation_status, topic_mocap_rigid_body,
    topic_object_state, topic_platform_command, topic_waypoint_intent, type_agent_state,
    type_authority_state, type_command_status, type_command_veto, type_constraint_event,
    type_frame_calibration, type_manipulation_command, type_manipulation_intent,
    type_manipulation_status, type_mocap_rigid_body, type_object_state, type_platform_command,
    type_waypoint_intent,
};
use crossbeam_channel::{Receiver, RecvTimeoutError};
use mcap::{Writer, records::MessageHeader};
use std::{
    collections::{BTreeMap, HashMap},
    error::Error,
    fs::{self, File},
    io::{BufWriter, Error as IoError, ErrorKind, Seek, Write},
    path::Path,
    sync::atomic::{AtomicBool, Ordering},
    time::Duration,
};

/// MCAP message encoding used for native protobuf payloads.
const PROTOBUF_MESSAGE_ENCODING: &str = "protobuf";

/// MCAP message encoding used for derived JSON payloads.
const JSON_MESSAGE_ENCODING: &str = "json";

/// Writer flush interval while waiting for new samples.
const FLUSH_INTERVAL_MS: u64 = 100;

/// Protobuf package prefix used in MCAP schema names.
const PROTO_PACKAGE: &str = "core_msgs";

/// Static registry of all channels this recorder knows how to write.
const RECORDED_CHANNELS: &[ChannelSpec] = &[
    ChannelSpec::protobuf(topic_waypoint_intent(), type_waypoint_intent()),
    ChannelSpec::protobuf(topic_agent_state(), type_agent_state()),
    ChannelSpec::protobuf(topic_constraint_event(), type_constraint_event()),
    ChannelSpec::protobuf(topic_authority_state(), type_authority_state()),
    ChannelSpec::protobuf(topic_platform_command(), type_platform_command()),
    ChannelSpec::protobuf(topic_command_veto(), type_command_veto()),
    ChannelSpec::protobuf(topic_command_status(), type_command_status()),
    ChannelSpec::protobuf(topic_mocap_rigid_body(), type_mocap_rigid_body()),
    ChannelSpec::protobuf(topic_frame_calibration(), type_frame_calibration()),
    ChannelSpec::protobuf(topic_manipulation_intent(), type_manipulation_intent()),
    ChannelSpec::protobuf(topic_manipulation_command(), type_manipulation_command()),
    ChannelSpec::protobuf(topic_manipulation_status(), type_manipulation_status()),
    ChannelSpec::protobuf(topic_object_state(), type_object_state()),
    ChannelSpec::json_schema(
        frame_transform::TOPIC_AGENT_TF,
        frame_transform::SCHEMA_FRAME_TRANSFORM,
        frame_transform::FRAME_TRANSFORM_JSON_SCHEMA,
    ),
];

/// Compile-time description of one MCAP channel and its schema.
#[derive(Clone, Copy, Debug)]
struct ChannelSpec {
    /// Topic/channel name.
    topic: &'static str,

    /// Short schema name before protobuf package qualification.
    schema_name: &'static str,

    /// MCAP schema encoding string, such as `protobuf` or `jsonschema`.
    schema_encoding: &'static str,

    /// Schema bytes registered with MCAP.
    schema: &'static [u8],

    /// Encoding string for messages written to this channel.
    message_encoding: &'static str,
}

impl ChannelSpec {
    /// Builds a protobuf channel using the shared descriptor set.
    const fn protobuf(topic: &'static str, schema_name: &'static str) -> Self {
        Self {
            topic,
            schema_name,
            schema_encoding: "protobuf",
            schema: core_msgs::proto::FILE_DESCRIPTOR_SET,
            message_encoding: PROTOBUF_MESSAGE_ENCODING,
        }
    }

    /// Builds a JSON channel using an inline JSON schema.
    const fn json_schema(
        topic: &'static str,
        schema_name: &'static str,
        schema: &'static str,
    ) -> Self {
        Self {
            topic,
            schema_name,
            schema_encoding: "jsonschema",
            schema: schema.as_bytes(),
            message_encoding: JSON_MESSAGE_ENCODING,
        }
    }
}

/// MCAP channel ID plus expected schema for validation at write time.
#[derive(Clone, Copy, Debug)]
struct RegisteredChannel {
    /// Channel ID assigned by the MCAP writer.
    id: u16,

    /// Schema name expected for samples routed to this topic.
    schema_name: &'static str,
}

/// Writes samples from the recorder channel into an MCAP file until shutdown.
pub fn write_mcap(
    path: &Path,
    rx: &Receiver<RecordedSample>,
    running: &AtomicBool,
) -> Result<(), Box<dyn Error>> {
    create_parent_dir(path)?;

    let file = BufWriter::new(File::create(path)?);
    let mut writer = Writer::new(file)?;
    let channels = add_channels(&mut writer)?;
    let mut sequence = 0;

    // Keep writing while Ctrl-C has not fired, then drain any samples already
    // forwarded by the DDS thread before finalizing the file.
    while running.load(Ordering::SeqCst) || !rx.is_empty() {
        match rx.recv_timeout(Duration::from_millis(FLUSH_INTERVAL_MS)) {
            Ok(sample) => write_sample(&mut writer, &channels, &mut sequence, &sample)?,
            Err(RecvTimeoutError::Timeout) => writer.flush()?,
            Err(RecvTimeoutError::Disconnected) => break,
        }
    }

    while let Ok(sample) = rx.try_recv() {
        write_sample(&mut writer, &channels, &mut sequence, &sample)?;
    }

    writer.finish()?;
    Ok(())
}

/// Creates the output directory if the requested path has one.
fn create_parent_dir(path: &Path) -> Result<(), IoError> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)?;
    }

    Ok(())
}

/// Registers all known schemas and channels before samples are written.
fn add_channels<W: Write + Seek>(
    writer: &mut Writer<W>,
) -> mcap::McapResult<HashMap<&'static str, RegisteredChannel>> {
    let mut channels = HashMap::new();

    for spec in RECORDED_CHANNELS {
        // MCAP expects protobuf schema names to include their package. JSON
        // schema names are already fully qualified for Foxglove.
        let schema_name = if spec.schema_encoding == "protobuf" {
            format!("{PROTO_PACKAGE}.{}", spec.schema_name)
        } else {
            spec.schema_name.to_owned()
        };
        let schema_id = writer.add_schema(&schema_name, spec.schema_encoding, spec.schema)?;
        let channel_id = writer.add_channel(
            schema_id,
            spec.topic,
            spec.message_encoding,
            &BTreeMap::new(),
        )?;
        channels.insert(
            spec.topic,
            RegisteredChannel {
                id: channel_id,
                schema_name: spec.schema_name,
            },
        );
    }

    Ok(channels)
}

/// Validates and writes one encoded sample to its registered MCAP channel.
fn write_sample<W: Write + Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, RegisteredChannel>,
    sequence: &mut u32,
    sample: &RecordedSample,
) -> Result<(), Box<dyn Error>> {
    let topic = sample.topic;
    let schema_name = sample.schema_name;
    let channel = channels.get(topic).ok_or_else(|| {
        IoError::new(
            ErrorKind::InvalidData,
            format!("no MCAP channel registered for topic `{topic}`"),
        )
    })?;
    // A mismatched schema/topic pair means the recorder thread sent a sample to
    // the wrong channel definition, which would make the MCAP hard to decode.
    if channel.schema_name != schema_name {
        return Err(IoError::new(
            ErrorKind::InvalidData,
            format!(
                "schema `{}` does not match registered schema `{}` for topic `{}`",
                schema_name, channel.schema_name, topic
            ),
        )
        .into());
    }

    writer.write_to_known_channel(
        &MessageHeader {
            channel_id: channel.id,
            sequence: *sequence,
            log_time: sample.log_time_ns,
            publish_time: sample.publish_time_ns,
        },
        &sample.payload,
    )?;
    *sequence = sequence.wrapping_add(1);

    Ok(())
}
