use super::*;
use std::{
    collections::VecDeque,
    io::{self, Cursor, ErrorKind, Read},
};

/// Builds the `NatNet` fields this parser consumes, including marker data that
/// proves the rigid-body cursor stays aligned.
fn frame_packet(rigid_bodies: &[(i32, [f32; 3], [f32; 4])]) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.extend_from_slice(&42i32.to_le_bytes());
    payload.extend_from_slice(&1i32.to_le_bytes());
    payload.extend_from_slice(b"alignment-markers\0");
    payload.extend_from_slice(&2i32.to_le_bytes());
    for value in [1.0f32, 2.0, 3.0, 4.0, 5.0, 6.0] {
        payload.extend_from_slice(&value.to_le_bytes());
    }
    payload.extend_from_slice(&1i32.to_le_bytes());
    for value in [-1.0f32, -2.0, -3.0] {
        payload.extend_from_slice(&value.to_le_bytes());
    }
    let rigid_body_count =
        i32::try_from(rigid_bodies.len()).expect("test packet body count fits i32");
    payload.extend_from_slice(&rigid_body_count.to_le_bytes());
    for (id, position, orientation) in rigid_bodies {
        payload.extend_from_slice(&id.to_le_bytes());
        for value in position {
            payload.extend_from_slice(&value.to_le_bytes());
        }
        for value in orientation {
            payload.extend_from_slice(&value.to_le_bytes());
        }
        payload.extend_from_slice(&0.001f32.to_le_bytes());
        payload.extend_from_slice(&1u16.to_le_bytes());
    }

    let mut packet = Vec::new();
    packet.extend_from_slice(&NATNET_FRAMEOFDATA.to_le_bytes());
    let payload_len = u16::try_from(payload.len()).expect("test packet payload fits u16");
    packet.extend_from_slice(&payload_len.to_le_bytes());
    packet.extend_from_slice(&payload);
    packet
}

#[test]
fn parses_rigid_bodies_after_marker_sets() {
    let packet = frame_packet(&[
        (7, [1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]),
        (9, [-1.5, 0.5, 9.0], [0.1, 0.2, 0.3, 0.9]),
    ]);

    let frame = parse_frame(&packet);
    let parsed = frame.as_slice();

    assert_eq!(parsed.len(), 2);
    assert_eq!(parsed[0].id, 7);
    assert_f32_slice_near(&parsed[0].position, &[1.0, 2.0, 3.0]);
    assert_f32_slice_near(&parsed[0].orientation, &[0.0, 0.0, 0.0, 1.0]);
    assert_eq!(parsed[1].id, 9);
    assert_f32_slice_near(&parsed[1].position, &[-1.5, 0.5, 9.0]);
}

#[test]
fn proto_conversion_preserves_pose_stamp_and_frame_contract() {
    let frame = parse_frame(&frame_packet(&[(
        -4,
        [1.0, -2.0, 3.5],
        [0.1, 0.2, 0.3, 0.9],
    )]));

    let message = frame.to_proto(12_345);

    assert_eq!(message.stamp_ms, 12_345);
    assert_eq!(message.frame_id, MOCAP_LAB_FRAME);
    assert_eq!(message.bodies.len(), 1);
    let body = &message.bodies[0];
    assert_eq!(body.id, -4);
    let position = body.position.as_ref().expect("position should be present");
    assert_f32_slice_near(&[position.x, position.y, position.z], &[1.0, -2.0, 3.5]);
    let orientation = body
        .orientation
        .as_ref()
        .expect("orientation should be present");
    assert_f32_slice_near(
        &[orientation.x, orientation.y, orientation.z, orientation.w],
        &[0.1, 0.2, 0.3, 0.9],
    );
}

#[test]
fn ignores_non_frame_messages() {
    let packet = [1u16.to_le_bytes(), 0u16.to_le_bytes()].concat();

    assert!(parse_frame(&packet).as_slice().is_empty());
}

#[test]
fn every_truncated_packet_is_rejected_without_panicking() {
    let full = frame_packet(&[(1, [1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])]);

    for cut in 0..full.len() {
        assert!(
            parse_frame(&full[..cut]).as_slice().is_empty(),
            "truncation at byte {cut} must not produce a partial observation"
        );
    }
}

#[test]
fn negative_counts_are_rejected_without_looping() {
    let mut packet = Vec::new();
    packet.extend_from_slice(&NATNET_FRAMEOFDATA.to_le_bytes());
    packet.extend_from_slice(&8u16.to_le_bytes());
    packet.extend_from_slice(&0i32.to_le_bytes());
    packet.extend_from_slice(&(-1i32).to_le_bytes());

    assert!(parse_frame(&packet).as_slice().is_empty());
}

#[test]
fn excess_rigid_bodies_are_consumed_but_storage_is_capped() {
    let body_count = u16::try_from(MAX_RIGID_BODIES + 5).expect("test body count fits u16");
    let bodies: Vec<_> = (0..body_count)
        .map(|id| (i32::from(id), [f32::from(id); 3], [0.0, 0.0, 0.0, 1.0]))
        .collect();

    let frame = parse_frame(&frame_packet(&bodies));

    assert_eq!(frame.as_slice().len(), MAX_RIGID_BODIES);
    assert_eq!(frame.as_slice().last().expect("last retained body").id, 63);
}

fn assert_f32_slice_near(actual: &[f32], expected: &[f32]) {
    assert_eq!(actual.len(), expected.len());
    for (actual, expected) in actual.iter().zip(expected) {
        assert!((actual - expected).abs() < f32::EPSILON);
    }
}

struct Chunked<R> {
    inner: R,
    max_chunk: usize,
}

impl<R: Read> Read for Chunked<R> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let limit = buf.len().min(self.max_chunk);
        self.inner.read(&mut buf[..limit])
    }
}

#[test]
fn read_full_reassembles_fragmented_stream_data() {
    let expected = b"length-prefixed-frame";
    let mut reader = Chunked {
        inner: Cursor::new(expected),
        max_chunk: 2,
    };
    let mut output = vec![0; expected.len()];
    let mut idle_calls = 0;

    read_full(&mut reader, &mut output, &mut || idle_calls += 1)
        .expect("fragmented reads should be reassembled");

    assert_eq!(output, expected);
    assert_eq!(idle_calls, 0);
}

enum ReadStep {
    Data(Vec<u8>),
    Error(ErrorKind),
    Eof,
}

struct ScriptedReader {
    steps: VecDeque<ReadStep>,
}

impl Read for ScriptedReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.steps.pop_front().unwrap_or(ReadStep::Eof) {
            ReadStep::Data(bytes) => {
                assert!(bytes.len() <= buf.len());
                buf[..bytes.len()].copy_from_slice(&bytes);
                Ok(bytes.len())
            }
            ReadStep::Error(kind) => Err(io::Error::from(kind)),
            ReadStep::Eof => Ok(0),
        }
    }
}

#[test]
fn read_full_tolerates_timeouts_and_interrupts_without_losing_alignment() {
    let mut reader = ScriptedReader {
        steps: VecDeque::from([
            ReadStep::Error(ErrorKind::TimedOut),
            ReadStep::Error(ErrorKind::Interrupted),
            ReadStep::Data(vec![1, 2]),
            ReadStep::Error(ErrorKind::WouldBlock),
            ReadStep::Data(vec![3, 4]),
        ]),
    };
    let mut output = [0; 4];
    let mut idle_calls = 0;

    read_full(&mut reader, &mut output, &mut || idle_calls += 1)
        .expect("recoverable read conditions should not abort the frame");

    assert_eq!(output, [1, 2, 3, 4]);
    assert_eq!(idle_calls, 1, "only a timeout before any bytes is idle");
}

#[test]
fn read_full_reports_closed_and_failed_connections() {
    let mut closed = Cursor::new([1u8, 2]);
    let mut output = [0; 3];
    let error = read_full(&mut closed, &mut output, &mut || {})
        .expect_err("EOF before a complete frame must be reported");
    assert_eq!(error.kind(), ErrorKind::UnexpectedEof);

    let mut failed = ScriptedReader {
        steps: VecDeque::from([ReadStep::Error(ErrorKind::ConnectionReset)]),
    };
    let error = read_full(&mut failed, &mut output, &mut || {})
        .expect_err("non-recoverable transport errors must be propagated");
    assert_eq!(error.kind(), ErrorKind::ConnectionReset);
}
