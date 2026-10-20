//! Motive `NatNet` 4.0 → Zenoh bridge.
//!
//! Ingests rigid-body frames one of two ways and republishes each as a
//! `mocap_rigid_body` (`MocapRigidBodyFrame`) message:
//!
//! - Direct UDP multicast from Motive, for native Linux hosts on the same
//!   network as the streaming workstation.
//! - A local TCP relay (`--mocap-wsl-relay`) fed by `natnet_relay.ps1` running
//!   on the Windows host. `WSL2` sits in an isolated network namespace and
//!   cannot receive the Windows-side UDP multicast directly, so the `PowerShell`
//!   relay forwards each packet over a length-prefixed TCP stream instead.

use core_msgs::{frames::MOCAP_LAB_FRAME, proto};
use prost::Message;
use std::time::Duration;
use zenoh::Wait;

// ── NatNet constants ──────────────────────────────────────────────────────────

/// `NatNet` message ID for `FrameOfData` packets.
const NATNET_FRAMEOFDATA: u16 = 7;

/// Maximum number of rigid bodies stored per frame.
const MAX_RIGID_BODIES: usize = 64;

/// Delay before retrying a dropped or failed relay connection.
const RECONNECT_DELAY: Duration = Duration::from_secs(1);

/// Read timeout on the UDP/TCP socket. A connected-but-silent Motive stream
/// surfaces as a periodic warning at this cadence.
const RELAY_READ_TIMEOUT: Duration = Duration::from_secs(5);

/// Frames between steady-state progress log lines (~5 s at Motive's 120 Hz).
const PROGRESS_LOG_FRAME_INTERVAL: u64 = 600;

/// Initial relay packet buffer size; grown on demand for larger frames.
const INITIAL_PACKET_BUFFER_BYTES: usize = 8192;

// ── Internal frame types ──────────────────────────────────────────────────────

/// One rigid-body pose parsed from a `NatNet` `FrameOfData` packet.
#[derive(Clone, Copy, Debug, Default)]
struct RigidBodyPose {
    id: i32,
    /// `NatNet` Y-up position: x = right, y = up (height), z = back.
    position: [f32; 3],
    /// Quaternion [x, y, z, w] as reported by Motive.
    orientation: [f32; 4],
}

/// Fixed-capacity collection of rigid-body poses from one `NatNet` frame.
struct RigidBodyFrame {
    bodies: [RigidBodyPose; MAX_RIGID_BODIES],
    len: usize,
}

impl Default for RigidBodyFrame {
    fn default() -> Self {
        Self {
            bodies: [RigidBodyPose::default(); MAX_RIGID_BODIES],
            len: 0,
        }
    }
}

impl RigidBodyFrame {
    fn as_slice(&self) -> &[RigidBodyPose] {
        &self.bodies[..self.len]
    }

    /// Converts the parsed frame into the shared protobuf message.
    fn to_proto(&self, stamp_ms: u64) -> proto::MocapRigidBodyFrame {
        proto::MocapRigidBodyFrame {
            bodies: self
                .as_slice()
                .iter()
                .map(|b| proto::RigidBodyPose {
                    id: b.id,
                    position: Some(proto::Vec3 {
                        x: b.position[0],
                        y: b.position[1],
                        z: b.position[2],
                    }),
                    orientation: Some(proto::Quat {
                        x: b.orientation[0],
                        y: b.orientation[1],
                        z: b.orientation[2],
                        w: b.orientation[3],
                    }),
                })
                .collect(),
            stamp_ms,
            frame_id: MOCAP_LAB_FRAME.to_owned(),
        }
    }
}

// ── NatNet packet parser ──────────────────────────────────────────────────────

/// Bounds-checked little-endian reader for `NatNet` packet buffers.
struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    const fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let end = self.pos.checked_add(n)?;
        let slice = self.buf.get(self.pos..end)?;
        self.pos = end;
        Some(slice)
    }

    fn read_u16(&mut self) -> Option<u16> {
        self.take(2).map(|b| u16::from_le_bytes([b[0], b[1]]))
    }

    fn read_i32(&mut self) -> Option<i32> {
        self.take(4)
            .and_then(|b| b.try_into().ok())
            .map(i32::from_le_bytes)
    }

    fn read_f32(&mut self) -> Option<f32> {
        self.take(4)
            .and_then(|b| b.try_into().ok())
            .map(f32::from_le_bytes)
    }

    /// Reads a null-terminated string and advances past the terminator.
    fn skip_cstr(&mut self) -> Option<()> {
        let rest = self.buf.get(self.pos..)?;
        let nul = rest.iter().position(|&b| b == 0)?;
        self.pos += nul + 1;
        Some(())
    }

    fn read_count(&mut self) -> Option<usize> {
        usize::try_from(self.read_i32()?).ok()
    }
}

/// Parses a `NatNet` 4.0 `FrameOfData` packet; returns an empty frame on error.
fn parse_frame(packet: &[u8]) -> RigidBodyFrame {
    try_parse_frame(packet).unwrap_or_default()
}

fn try_parse_frame(packet: &[u8]) -> Option<RigidBodyFrame> {
    let mut r = Reader::new(packet);

    // NatNet packet header.
    let message_id = r.read_u16()?;
    let _payload_len = r.read_u16()?;
    if message_id != NATNET_FRAMEOFDATA {
        return None;
    }
    let _frame_number = r.read_i32()?;

    // Marker sets (count = 0 when only Rigid Bodies are enabled in Motive).
    let n_marker_sets = r.read_count()?;
    for _ in 0..n_marker_sets {
        r.skip_cstr()?; // marker set name
        let n_markers = r.read_count()?;
        r.take(n_markers.checked_mul(12)?)?; // 3 × f32 per marker
    }

    // Unlabeled markers (count = 0 when only Rigid Bodies are enabled).
    let n_unlabeled = r.read_count()?;
    r.take(n_unlabeled.checked_mul(12)?)?;

    // Rigid bodies — the data we actually want.
    let n_rigid_bodies = r.read_count()?;
    let mut frame = RigidBodyFrame::default();
    for _ in 0..n_rigid_bodies {
        let id = r.read_i32()?;
        let position = [r.read_f32()?, r.read_f32()?, r.read_f32()?];
        let orientation = [r.read_f32()?, r.read_f32()?, r.read_f32()?, r.read_f32()?];

        let _mean_error = r.read_f32()?;
        let _tracking_params = r.read_u16()?;

        if frame.len < MAX_RIGID_BODIES {
            frame.bodies[frame.len] = RigidBodyPose {
                id,
                position,
                orientation,
            };
            frame.len += 1;
        }
    }

    Some(frame)
}

// ── Entry point ───────────────────────────────────────────────────────────────

fn publish_frame(
    packet: &[u8],
    publisher: &zenoh::pubsub::Publisher<'_>,
    frames_received: &mut u64,
    last_log_frame: &mut u64,
    via_tcp: bool,
) {
    let frame = parse_frame(packet);
    let n_bodies = frame.as_slice().len();
    if n_bodies == 0 {
        return;
    }

    let msg = frame.to_proto(core_msgs::now_ms());
    if let Err(e) = publisher.put(msg.encode_to_vec()).wait() {
        eprintln!("mocap_bridge: publish error: {e}");
    }

    *frames_received += 1;
    if *frames_received == 1 {
        let prefix = if via_tcp {
            "via TCP relay — "
        } else {
            "— "
        };
        println!(
            "mocap_bridge: first frame {prefix}{n_bodies} rigid bod{}.",
            if n_bodies == 1 { "y" } else { "ies" }
        );
    } else if *frames_received - *last_log_frame >= PROGRESS_LOG_FRAME_INTERVAL {
        println!(
            "mocap_bridge: {} frames published, {} rigid bod{} per frame.",
            *frames_received,
            n_bodies,
            if n_bodies == 1 { "y" } else { "ies" }
        );
        *last_log_frame = *frames_received;
    }
}

struct MocapArgs {
    ip: String,
    port: u16,
    wsl_relay: bool,
}

fn parse_args(args: impl IntoIterator<Item = std::ffi::OsString>) -> MocapArgs {
    let mut mocap_ip = "239.255.42.99".to_owned();
    let mut mocap_port = 1511;
    let mut mocap_wsl_relay = false;

    let args: Vec<std::ffi::OsString> = args.into_iter().collect();
    let mut i = 0;
    while i < args.len() {
        if let Some(arg_str) = args[i].to_str() {
            match arg_str {
                "--mocap-ip" => {
                    if let Some(val) = args.get(i + 1).and_then(|a| a.to_str()) {
                        val.clone_into(&mut mocap_ip);
                    }
                    i += 2;
                }
                "--mocap-port" => {
                    if let Some(val) = args.get(i + 1).and_then(|a| a.to_str())
                        && let Ok(p) = val.parse::<u16>()
                    {
                        mocap_port = p;
                    }
                    i += 2;
                }
                "--mocap-wsl-relay" => {
                    mocap_wsl_relay = true;
                    i += 1;
                }
                _ => i += 1,
            }
        } else {
            i += 1;
        }
    }

    MocapArgs {
        ip: mocap_ip,
        port: mocap_port,
        wsl_relay: mocap_wsl_relay,
    }
}

/// Runs the NatNet-to-Zenoh bridge with command-line arguments.
///
/// # Errors
///
/// Returns an error when the UDP socket, multicast membership, or Zenoh
/// publisher cannot be initialized.
pub fn run_with_args(
    args: impl IntoIterator<Item = std::ffi::OsString>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let MocapArgs {
        ip: mocap_ip,
        port: mocap_port,
        wsl_relay: mocap_wsl_relay,
    } = parse_args(args);

    let zenoh_session = dds_support::open_session();
    let publisher =
        dds_support::declare_publisher(&zenoh_session, core_msgs::topic_mocap_rigid_body());

    if mocap_wsl_relay {
        let relay_addr = "127.0.0.1:1512";
        println!(
            "mocap_bridge: connecting to Windows TCP relay at {relay_addr} \
             (run natnet_relay.ps1 on Windows first)…"
        );

        loop {
            match std::net::TcpStream::connect(relay_addr) {
                Ok(mut stream) => {
                    println!("mocap_bridge: connected to TCP relay.");
                    run_session(&mut stream, &publisher);
                    eprintln!("mocap_bridge: TCP relay disconnected; reconnecting…");
                }
                Err(e) => {
                    eprintln!(
                        "mocap_bridge: cannot reach relay {relay_addr} ({e}). \
                         Is natnet_relay.ps1 running on Windows? Retrying…"
                    );
                }
            }
            std::thread::sleep(RECONNECT_DELAY);
        }
    }

    println!("mocap_bridge: starting on UDP {mocap_ip}:{mocap_port}…");

    let addr = std::net::SocketAddr::from(([0, 0, 0, 0], mocap_port));
    let socket = std::net::UdpSocket::bind(addr)?;

    if let Ok(ip) = mocap_ip.parse::<std::net::Ipv4Addr>()
        && ip.is_multicast()
    {
        socket.join_multicast_v4(&ip, &std::net::Ipv4Addr::UNSPECIFIED)?;
        println!("mocap_bridge: joined multicast group {ip}");
    }

    socket.set_read_timeout(Some(RELAY_READ_TIMEOUT))?;

    let mut packet = vec![0u8; INITIAL_PACKET_BUFFER_BYTES];
    let mut frames_received: u64 = 0;
    let mut last_log_frame: u64 = 0;
    let mut idle_secs: u64 = 0;

    loop {
        match socket.recv_from(&mut packet) {
            Ok((len, _)) => {
                idle_secs = 0;
                publish_frame(
                    &packet[..len],
                    &publisher,
                    &mut frames_received,
                    &mut last_log_frame,
                    false,
                );
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                idle_secs += RELAY_READ_TIMEOUT.as_secs();
                eprintln!(
                    "mocap_bridge: listening on {mocap_ip}:{mocap_port}, but no NatNet data for {idle_secs}s — \
                     is Motive streaming (Rigid Bodies on)?"
                );
            }
            Err(e) => {
                eprintln!("mocap_bridge: UDP read error: {e}");
                std::thread::sleep(std::time::Duration::from_secs(1));
            }
        }
    }
}

/// Reads exactly `buf.len()` bytes, tolerating read timeouts.
fn read_full<R: std::io::Read>(
    reader: &mut R,
    buf: &mut [u8],
    on_idle: &mut impl FnMut(),
) -> std::io::Result<()> {
    use std::io::ErrorKind;

    let mut filled = 0;
    while filled < buf.len() {
        match reader.read(&mut buf[filled..]) {
            Ok(0) => {
                return Err(std::io::Error::new(
                    ErrorKind::UnexpectedEof,
                    "relay closed the connection",
                ));
            }
            Ok(n) => filled += n,
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {
                if filled == 0 {
                    on_idle();
                }
            }
            Err(e) if e.kind() == ErrorKind::Interrupted => {}
            Err(e) => return Err(e),
        }
    }
    Ok(())
}

/// Reads the `PowerShell` relay's TCP stream, where each `NatNet` packet is
/// framed by a 4-byte little-endian length prefix: read that length, then the
/// payload of exactly that many bytes.
fn run_session(stream: &mut std::net::TcpStream, publisher: &zenoh::pubsub::Publisher<'_>) {
    if let Err(e) = stream.set_read_timeout(Some(RELAY_READ_TIMEOUT)) {
        eprintln!("mocap_bridge: could not set relay read timeout: {e}");
    }

    let mut length = [0u8; 4];
    let mut packet = vec![0u8; INITIAL_PACKET_BUFFER_BYTES];
    let mut frames_received: u64 = 0;
    let mut last_log_frame: u64 = 0;
    let mut idle_secs: u64 = 0;

    loop {
        let mut on_idle = || {
            idle_secs += RELAY_READ_TIMEOUT.as_secs();
            eprintln!(
                "mocap_bridge: connected to TCP relay, but no NatNet data for {idle_secs}s — \
                 is Motive streaming?"
            );
        };
        if let Err(e) = read_full(stream, &mut length, &mut on_idle) {
            eprintln!("mocap_bridge: relay read error: {e}");
            return;
        }
        idle_secs = 0;

        let len = u32::from_le_bytes(length) as usize;
        if len > packet.len() {
            packet.resize(len, 0);
        }
        if let Err(e) = read_full(stream, &mut packet[..len], &mut || {}) {
            eprintln!("mocap_bridge: relay read error: {e}");
            return;
        }

        publish_frame(
            &packet[..len],
            publisher,
            &mut frames_received,
            &mut last_log_frame,
            true,
        );
    }
}

#[cfg(test)]
#[path = "../../../tests/unit/mocap_zenoh_bridge.rs"]
mod mocap_zenoh_bridge_tests;
