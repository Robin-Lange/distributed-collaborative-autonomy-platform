# MOCAP operation

The MOCAP path carries OptiTrack Motive rigid-body poses into the Zenoh bus. It is used for Spot frame registration and for objects whose catalog entry contains a `mocap_rigid_body_id`.

The launcher starts the bridge and object tracker when at least one agent uses a `hardware` or `hybrid` backend. The TUI waits for both a MOCAP frame and platform state before entering the operator screen.

## Data path

```text
Motive NatNet
  -> direct UDP or Windows TCP relay
  -> mocap_zenoh_bridge
  -> mocap_rigid_body
  -> object_tracker and Spot calibration
  -> object_state / frame_calibration
```

`sensors/mocap/src/bridge.rs` parses bounds-checked NatNet 4 `FrameOfData` packets. A malformed or truncated datagram is discarded without publishing a partial frame.

`sensors/mocap/src/tracker.rs` matches rigid-body streaming IDs to the object catalog, transforms poses from `mocap_lab` to ENU `map`, and publishes object state. A tracked object becomes invalid after 300 ms without a valid matching sample.

## Motive setup

Enable data streaming in Motive and configure:

- NatNet 4 compatible output;
- rigid bodies enabled;
- a unique streaming ID for each tracked robot or object;
- skeletons, marker sets, and unlabeled markers disabled unless another consumer needs them;
- the correct local network interface.

Every tracked object ID must match `mocap_rigid_body_id` in the TUI object setup. The Spot rigid-body ID is configured separately on the Spot Sidecar tab.

## Direct UDP

Use `Direct (UDP)` in Global Settings when the launcher host can receive Motive's UDP stream.

The defaults are:

```text
Multicast group: 239.255.42.99
Data port:       1511
```

Set Motive to multicast on the network interface that reaches the launcher host. Enter the same multicast group and data port in the TUI.

If the bridge reports that it joined the group but receives no frames:

1. Confirm Motive is streaming and the rigid body is actively tracked.
2. Confirm the selected Motive interface is on the same network as the launcher.
3. Check the host firewall for inbound UDP on the data port.
4. Check that another application has not taken exclusive ownership of the port.

## Windows and WSL relay

WSL2 may not receive the Windows host's inbound NatNet UDP stream reliably. The repository includes `sensors/mocap/natnet_relay.ps1`, which receives NatNet on Windows and serves length-prefixed packets to WSL over TCP port 1512.

In the TUI, select `WSL Relay (TCP)`. The Rust bridge then connects to:

```text
127.0.0.1:1512
```

Run the relay from Windows PowerShell before starting the stack.

For the same Windows machine, unicast is the default and is usually the simplest setup:

```powershell
powershell -ExecutionPolicy Bypass -File .\sensors\mocap\natnet_relay.ps1
```

Configure Motive for unicast on loopback with rigid bodies enabled.

For multicast:

```powershell
powershell -ExecutionPolicy Bypass `
  -File .\sensors\mocap\natnet_relay.ps1 `
  -Mode multicast `
  -McastIface 198.51.100.191
```

Replace the interface address with the Windows address selected in Motive. Keep the relay TCP port at 1512; the current Rust relay client uses that fixed local endpoint.

The relay accepts multiple clients. A standalone diagnostic bridge and the launcher-managed bridge can therefore consume the same Motive stream.

## Spot calibration

With Auto Calibrate enabled, the Spot sidecar combines:

- the configured static `T_map_mocap` transform;
- the selected Spot MOCAP rigid-body pose;
- the configured translation from that rigid-body origin to Spot's body frame;
- Spot's stationary vision-to-body transform.

The result is a locked map-to-vision registration. After calibration, live Spot localization comes from Spot odometry composed with that registration. MOCAP continues to track configured objects.

The three `Mocap -> Body` fields on the Spot Sidecar setup tab are the measured translation from the Motive rigid-body origin to Spot's body origin. Enter the vector in Spot body axes: forward, left, and up, in meters. The default is zero, which preserves the old behavior when the Motive origin already represents the body origin. The sidecar rotates the horizontal offset with Spot's measured MOCAP yaw and applies it once during registration. Its calibration log records the marker pose, configured offset, and corrected body pose.

Keep Spot stationary and fully visible while calibration locks. Measure this offset for the actual rigid-body definition instead of copying a value from another marker layout. If calibration does not complete:

- verify the Spot body ID in the TUI;
- verify the rigid-body-to-Spot-body translation;
- verify that ID appears in the incoming frame;
- confirm the static MOCAP-to-map transform is correct;
- confirm Spot state is arriving;
- remove marker occlusions and retry startup.

## Tracked-object behavior

The object tracker owns pose validity for MOCAP objects. It also follows manipulation status:

- a successful pick marks the object held by the commanding agent;
- a successful place marks it free;
- a failed manipulation leaves ownership unchanged.

The agent does not publish behavior success until this lifecycle update arrives. A Spot success without the matching ownership transition becomes an execution failure after a short confirmation timeout.

In hybrid operation, Avian mirrors a held object's pose from every live platform-state update. MOCAP reacquisition after placement resumes normal tracked-object updates.

## Connection screen

The connection screen reports MOCAP, agent state, bridge output, child-process state, and a failure reason.

- A timeout can be retried after correcting a missing stream.
- A required child-process exit is not treated as a timeout; exit and relaunch after correcting the reported startup error.
- Simulation-only launches do not start or wait for MOCAP.

## Useful checks

The bridge logs the first frames and periodic frame counts. During diagnosis, look for:

- `connected to TCP relay` in relay mode;
- `joined multicast group` in direct mode;
- a nonzero rigid-body count;
- an object ID that matches the configured catalog;
- `pose_valid` becoming true in the TUI or recorder.

Parser and relay-read behavior is covered by `tests/unit/mocap_zenoh_bridge.rs`. Tracked-object ownership transitions are covered by `tests/unit/mocap_tracker.rs`.
