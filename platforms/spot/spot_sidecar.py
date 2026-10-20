"""Stateless Spot <-> decentralized_autonomy bus sidecar.

Bridges Zenoh pub/sub (protobuf, via generated core_msgs_pb2) to Boston
Dynamics SDK calls and back. Holds no goal state of its own: goal/decision
logic lives in the Rust onboard_agent; this process only translates approved
commands into SDK actuation and republishes Spot's live state and faults.

Real mode runs no environment_sim process (the "no virtual walls" design), so
with --owns-object-state this sidecar is the sole authority for the virtual
object catalog.

Threading: zenoh delivers the on_* callbacks on its own subscriber threads, a
single 40 Hz step() loop publishes state and drains parked commands, and each
manipulation task runs on its own daemon thread. self._lock (reentrant) guards
command and actuation state.
"""

import core_msgs_pb2 as pb


import argparse
import collections
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # core_msgs_pb2

import zenoh

import queue

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    FOOT_FRAME_NAMES,
    GRAV_ALIGNED_BODY_FRAME_NAME,
    GROUND_PLANE_FRAME_NAME,
    VISION_FRAME_NAME,
    get_a_tform_b,
)
from bosdyn.client.lease import (
    LeaseClient,
    LeaseKeepAlive,
    ResourceAlreadyClaimedError,
)
from bosdyn.client.power import KeepaliveMotorsOffError
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    blocking_stand,
)
from bosdyn.client.robot_state import RobotStateClient

NAMESPACE = "decentralized_autonomy"
MULTICAST_INTERFACE_ENV = "DECENTRALIZED_AUTONOMY_MULTICAST_IFACE"
CONTROL_STDIN_ENV = "DCA_SPOT_CONTROL_STDIN"
STATE_HZ = 40.0
# SDK command end-time applied to every velocity send: Spot halts on its own if
# a fresh setpoint isn't dispatched within this window, so the setpoint stream
# itself is the runaway keepalive (no separate watchdog).
VELOCITY_CMD_EXPIRATION_S = 0.6
MAP_FRAME = "map"
CHILD_FRAME_SPOT_BODY = "spot_body"
APPROVED_BY = "onboard_agent"
FAULT_REASSERT_CYCLES = 4

SPOT_BODY_LENGTH_M = 1.1
SPOT_BODY_HEIGHT_M = 0.61

CAL_WINDOW_S = 2.0  # stationary window the mocap fix is averaged over
CAL_MIN_SAMPLES = 20  # minimum mocap frames inside the window
CAL_MAX_POS_SPREAD_M = 0.01  # max deviation from the window's mean position
CAL_MAX_YAW_SPREAD_RAD = math.radians(0.5)
CAL_MAX_STALE_S = 0.5  # newest mocap sample must be at most this old
CAL_WAIT_LOG_PERIOD_S = 5.0


def topic(name: str) -> str:
    return f"{NAMESPACE}/{name}"


def _zenoh_config(multicast_interface: str | None = None):
    value = (
        os.environ.get(MULTICAST_INTERFACE_ENV, "auto")
        if multicast_interface is None
        else multicast_interface
    ).strip()
    config = zenoh.Config()
    config.insert_json5("mode", '"peer"')
    config.insert_json5("listen/endpoints", '{peer: ["tcp/0.0.0.0:0"]}')
    if value.lower() == "disabled":
        config.insert_json5("scouting/multicast/enabled", "false")
    else:
        config.insert_json5(
            "scouting/multicast/interface", json.dumps(value or "auto")
        )
    return config


def _open_zenoh_session():
    config = _zenoh_config()
    try:
        return zenoh.open(config)
    except Exception as first_error:
        if (
            os.environ.get(MULTICAST_INTERFACE_ENV, "auto")
            .strip()
            .lower()
            == "disabled"
        ):
            raise
        print(
            "sidecar: Zenoh multicast startup failed "
            f"({first_error}); retrying with multicast scouting disabled"
        )
        return zenoh.open(_zenoh_config("disabled"))


def now_ms() -> int:
    return int(time.time() * 1000)


def _wait_for_control_input_close(stream, shutdown_requested) -> None:
    try:
        for _line in stream:
            pass
    finally:
        shutdown_requested.set()


def _start_shutdown_listener():
    shutdown_requested = threading.Event()
    if os.environ.get(CONTROL_STDIN_ENV) == "1":
        threading.Thread(
            target=_wait_for_control_input_close,
            args=(sys.stdin, shutdown_requested),
            daemon=True,
        ).start()
    return shutdown_requested


def _control_source(source: str) -> int:
    if source == "Autonomy":
        return pb.COMMAND_SOURCE_AUTONOMY
    if source == "Operator":
        return pb.COMMAND_SOURCE_HUMAN_OPERATOR
    return pb.COMMAND_SOURCE_NONE


def _control_position(target: dict) -> pb.Vec3:
    if "Planar" in target:
        x, y = target["Planar"]
        return pb.Vec3(x=x, y=y, z=0.0)
    x, y, z = target["Spatial"]
    return pb.Vec3(x=x, y=y, z=z)






import numpy as np
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.api import arm_command_pb2, geometry_pb2, gripper_command_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2

VELOCITY_LIMIT_MPS = 1.5  # robot-side SDK bound (SE2VelocityLimit); the onboard PD
# controller (autonomy/guidance.rs) clamps every request to this same
# value before it ever reaches the bus, since Spot rejects (rather than
# clamps) a request outside the vel_limit declared below.

# Built via the SDK helper (not a bare MobilityParams) so locomotion_hint defaults
# to HINT_AUTO — a bare proto leaves it HINT_UNKNOWN, which stops Spot producing a
# gait for synchro_velocity_command. Only the vel_limit is overridden below.
_VELOCITY_LIMIT_PARAMS = RobotCommandBuilder.mobility_params()
_VELOCITY_LIMIT_PARAMS.vel_limit.CopyFrom(
    geometry_pb2.SE2VelocityLimit(
        max_vel=geometry_pb2.SE2Velocity(
            linear=geometry_pb2.Vec2(x=VELOCITY_LIMIT_MPS, y=VELOCITY_LIMIT_MPS),
            angular=VELOCITY_LIMIT_MPS,
        ),
        min_vel=geometry_pb2.SE2Velocity(
            linear=geometry_pb2.Vec2(x=-VELOCITY_LIMIT_MPS, y=-VELOCITY_LIMIT_MPS),
            angular=-VELOCITY_LIMIT_MPS,
        ),
    )
)

def _quat_to_rot3(qx: float, qy: float, qz: float, qw: float) -> list:
    q = Quat(w=float(qw), x=float(qx), y=float(qy), z=float(qz)).normalize()
    m = q.to_matrix()
    return [[m[i][j] for j in range(3)] for i in range(3)]

def _rot3_to_quat(r: list) -> tuple:
    q = Quat.from_matrix(np.array(r, dtype=float))
    return q.x, q.y, q.z, q.w

def _se3_to_mat4(rot3: list, t: tuple) -> list:
    r = rot3
    return [
        r[0][0], r[0][1], r[0][2], t[0],
        r[1][0], r[1][1], r[1][2], t[1],
        r[2][0], r[2][1], r[2][2], t[2],
        0.0, 0.0, 0.0, 1.0,
    ]

def _mat4_to_se3pose(m: list) -> SE3Pose:
    return SE3Pose.from_matrix(np.array(m, dtype=float).reshape(4, 4))

def _se3_pose_to_mat4(pose) -> list:
    if hasattr(pose, 'to_matrix'):
        return [float(v) for v in pose.to_matrix().reshape(-1)]
    return _se3_to_mat4(
        _quat_to_rot3(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w),
        (pose.x, pose.y, pose.z),
    )

def _mat4_compose(a: list, b: list) -> list:
    return _se3_pose_to_mat4(_mat4_to_se3pose(a).mult(_mat4_to_se3pose(b)))

def _mat4_apply_point(m: list, p: tuple) -> tuple:
    return _mat4_to_se3pose(m).transform_point(*p)

def _mat4_apply_vector(m: list, v: tuple) -> tuple:
    return _mat4_to_se3pose(m).rot.transform_point(*v)

def _mat4_from_yaw_t(theta: float, t: tuple) -> list:
    return _se3_pose_to_mat4(SE3Pose(t[0], t[1], t[2], Quat.from_yaw(theta)))

def _mat4_from_yaw_t_inv(theta: float, t: tuple) -> list:
    return _se3_pose_to_mat4(SE3Pose(t[0], t[1], t[2], Quat.from_yaw(theta)).inverse())


def _spot_body_position_map(
    mocap_origin_map: tuple,
    body_yaw_map: float,
    mocap_to_body_translation_m: tuple,
) -> tuple:
    """Apply the configured Motive-origin to Spot-BODY translation."""

    forward_m, left_m, up_m = mocap_to_body_translation_m
    cos_yaw = math.cos(body_yaw_map)
    sin_yaw = math.sin(body_yaw_map)
    return (
        mocap_origin_map[0] + cos_yaw * forward_m - sin_yaw * left_m,
        mocap_origin_map[1] + sin_yaw * forward_m + cos_yaw * left_m,
        mocap_origin_map[2] + up_m,
    )


def _map_vision_registration(
    body_position_map: tuple,
    body_yaw_map: float,
    body_pose_vision,
):
    """Register Spot's vision frame to the map from one shared body pose."""

    t_map_body = _mat4_from_yaw_t(body_yaw_map, body_position_map)
    t_body_map = _mat4_from_yaw_t_inv(body_yaw_map, body_position_map)
    t_body_vision = _se3_pose_to_mat4(body_pose_vision.inverse())
    t_vision_body = _se3_pose_to_mat4(body_pose_vision)
    return (
        t_map_body,
        t_body_map,
        _mat4_compose(t_map_body, t_body_vision),
        _mat4_compose(t_vision_body, t_body_map),
    )


ARM_PHASE_TIMEOUT_S = 10.0
HOVER_OFFSET_M = 0.12
ARM_MOVE_SECONDS = 2.0
FLOOR_CLEARANCE_M = 0.05
# The jaw tips share this wrist-frame X coordinate. Z=0 places the tool
# halfway between the upper and lower jaws instead of on either jaw surface.
GRIP_CENTER_X_FROM_WRIST_M = 0.23589

_WRIST_TFORM_GRIP_CENTER = SE3Pose(
    GRIP_CENTER_X_FROM_WRIST_M,
    0.0,
    0.0,
    Quat.from_pitch(-math.pi / 2.0),
)

_ARM_BODY_ASSIST_PARAMS = spot_command_pb2.MobilityParams(
    body_control=spot_command_pb2.BodyControlParams(
        body_assist_for_manipulation=spot_command_pb2.BodyControlParams.BodyAssistForManipulation(
            enable_hip_height_assist=True,
            enable_body_yaw_assist=False,
        )
    )
)


class ArmSafetyStop(RuntimeError):
    """Arm execution was interrupted by the platform safety path."""


class NoObjectDetected(RuntimeError):
    """Spot fully closed the gripper without detecting contact."""


class UnsafeGripHeight(RuntimeError):
    """The exact requested grip center violates the floor-clearance invariant."""


def _grip_pose_in_vision(
    pos_map: tuple,
    yaw_rad: float,
    t_vision_map: list,
    t_vision_ground,
):
    """Transform an exact map-frame grip target and validate floor clearance."""

    if not all(math.isfinite(value) for value in (*pos_map, yaw_rad)):
        raise ValueError("requested grip pose contains non-finite values")

    t_map_grip = SE3Pose(
        x=pos_map[0],
        y=pos_map[1],
        z=pos_map[2],
        rot=Quat.from_yaw(yaw_rad),
    )
    t_vision_grip = _mat4_to_se3pose(t_vision_map).mult(t_map_grip)
    t_ground_grip = t_vision_ground.inverse().mult(t_vision_grip)
    requested_height = t_ground_grip.z
    if requested_height < FLOOR_CLEARANCE_M:
        raise UnsafeGripHeight(
            f"requested grip point is {requested_height:.3f} m above ground; "
            f"safe floor clearance requires at least {FLOOR_CLEARANCE_M:.3f} m"
        )
    return t_vision_grip, requested_height


def _grip_pose_command(
    pos_map: tuple,
    yaw_rad: float,
    t_vision_map: list,
    t_vision_ground,
):
    t_vision_grip, requested_height = _grip_pose_in_vision(
        pos_map,
        yaw_rad,
        t_vision_map,
        t_vision_ground,
    )
    command = RobotCommandBuilder.arm_pose_command_from_pose(
        t_vision_grip.to_proto(),
        VISION_FRAME_NAME,
        ARM_MOVE_SECONDS,
    )
    command.synchronized_command.arm_command.arm_cartesian_command.wrist_tform_tool.CopyFrom(
        _WRIST_TFORM_GRIP_CENTER.to_proto()
    )
    return (
        RobotCommandBuilder.synchro_stand_command(
            params=_ARM_BODY_ASSIST_PARAMS,
            build_on_command=command,
        ),
        requested_height,
    )


def _arm_cartesian_status(response):
    feedback = (
        response.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback
    )
    status = arm_command_pb2.ArmCartesianCommand.Feedback
    if feedback.status == status.STATUS_TRAJECTORY_COMPLETE:
        return "COMPLETE"
    if feedback.status == status.STATUS_TRAJECTORY_STALLED:
        return "FAILED: Cartesian trajectory stalled"
    if feedback.status == status.STATUS_TRAJECTORY_CANCELLED:
        return "FAILED: Cartesian trajectory cancelled"
    return "PENDING"


def _map_heading_from_vision_pose(t_map_vision: list, tform) -> float:
    """Map-frame heading yaw (rad) of the body from its vision-frame pose.

    Forward is the body +X axis: rotate it through the vision pose, then into
    map, and take atan2(y, x).
    """

    fwd_vision = _mat4_apply_vector(_se3_pose_to_mat4(tform), (1.0, 0.0, 0.0))
    fwd_x, fwd_y, _fwd_z = _mat4_apply_vector(t_map_vision, fwd_vision)
    return math.atan2(fwd_y, fwd_x)


def _mocap_yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """Spot's map-frame heading yaw (rad) from its Motive rigid-body quaternion.

    Single source of truth for the Spot mocap mounting convention: a wrong sign
    or axis here silently miscalibrates every downstream pose.

    The rigid body is mounted so its local +Z points along Spot's forward
    heading. Rotating +Z by the reported orientation gives that heading as the
    rotation matrix's third column, expressed in NatNet's Y-up world frame
    (x=right, y=up, z=back; see mocap/bridge.rs). The atan2 arguments project
    that axis onto the ground plane and encode the NatNet Y-up -> map ENU flip,
    so the result is already in the map yaw convention -- on_mocap relies on
    this, transforming only position through t_map_mocap and never the yaw.
    """

    r = _quat_to_rot3(qx, qy, qz, qw)
    return math.atan2(r[2][2], -r[0][2])


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _pose_window_stats(poses: list):
    """Mean pose and worst-case spread over a window of (x, y, z, yaw) samples.

    Yaw uses a circular mean (atan2 of summed sin/cos) so the wrap at +/-pi
    doesn't corrupt it; each spread is the max deviation of any sample from the
    mean, used to gate auto-calibration on stationarity.
    """

    n = len(poses)
    mx = sum(p[0] for p in poses) / n
    my = sum(p[1] for p in poses) / n
    mz = sum(p[2] for p in poses) / n
    myaw = math.atan2(
        sum(math.sin(p[3]) for p in poses), sum(math.cos(p[3]) for p in poses)
    )
    pos_spread = max(
        math.sqrt((p[0] - mx) ** 2 + (p[1] - my) ** 2 + (p[2] - mz) ** 2) for p in poses
    )
    yaw_spread = max(abs(_wrap_pi(p[3] - myaw)) for p in poses)
    return mx, my, mz, myaw, pos_spread, yaw_spread


def _load_t_map_mocap(path: str):

    env_json = os.environ.get("DCA_CONFIG_CONFIG_FRAMES_JSON")
    if env_json:
        entry = json.loads(env_json)["frames"]["mocap_lab_to_map"]
    else:
        with open(path, encoding="utf-8") as f:
            entry = json.load(f)["frames"]["mocap_lab_to_map"]
    return entry["rotation_matrix_rows"], entry.get("translation_m", [0.0, 0.0, 0.0])


def _load_object_catalog_seeds(path: str) -> list:

    env_json = os.environ.get("DCA_CONFIG_CONFIG_OBJECTS_JSON")
    if env_json:
        objects = json.loads(env_json)["objects"]
    else:
        with open(path, encoding="utf-8") as f:
            objects = json.load(f)["objects"]
    return [
        (
            entry["object_id"],
            entry["virtual_initial_pose_map"]["x"],
            entry["virtual_initial_pose_map"]["y"],
            entry["virtual_initial_pose_map"]["z"],
            entry["virtual_initial_pose_map"]["yaw_rad"],
        )
        for entry in objects
        if entry.get("mocap_rigid_body_id") is None
    ]


def _command_rejection(cmd, agent_id: str):
    """Reason to drop `cmd`, or None if it is authorized for this sidecar.

    Authorization gate for every inbound command: a command is accepted only if
    it is addressed to this sidecar's agent_id and carries
    approved_by == "onboard_agent". Anything else is rejected with a
    human-readable reason string.
    """

    if cmd.agent_id != agent_id:
        return f"agent_id={cmd.agent_id!r} (this sidecar is {agent_id!r})"
    if cmd.approved_by != APPROVED_BY:
        return f"approved_by={cmd.approved_by!r} (want {APPROVED_BY!r})"
    return None


def _may_replace_pending(pending, new) -> bool:
    """Whether `new` may overwrite the parked `pending` command (latest-wins).

    A parked HOLD/STOP is sticky: it is replaced only by another HOLD/STOP or by
    a strictly newer setpoint, so a stale velocity can never bump a pending
    safety stop out of the queue.
    """

    if pending is None or (pending.command_type != pb.COMMAND_TYPE_HOLD and pending.command_type != pb.COMMAND_TYPE_STOP):
        return True
    if new.command_type in (pb.COMMAND_TYPE_HOLD, pb.COMMAND_TYPE_STOP):
        return True
    return new.stamp_ms > pending.stamp_ms


def _fault_condition_ids(state) -> set:
    """Stable ids of the currently-active fault conditions in `state`.

    Covers unavailable motor power, e-stop, every behavior fault, and only
    SEVERITY_CRITICAL system faults; these ids are what _publish_fault
    level-triggers on.
    """

    conditions = set()
    if state.power_state.motor_power_state != state.power_state.STATE_ON:
        conditions.add("motors_off")
    if any(s.state == s.STATE_ESTOPPED for s in state.estop_states):
        conditions.add("estop")
    for fault in state.behavior_fault_state.faults:
        conditions.add(f"behavior_fault:{fault.behavior_fault_id}")
    for fault in state.system_fault_state.faults:
        if fault.severity == fault.SEVERITY_CRITICAL:
            conditions.add(f"system_fault:{fault.name}")
    return conditions


def _fault_transitions(active: set, previous: set, reassert: bool) -> list:
    """(condition_id, is_hard_stop) events driving the fault level-trigger.

    Newly-active conditions, and (when `reassert`) still-active ones, emit a
    hard-stop event; cleared conditions emit a single recovery event. Sorted for
    deterministic publish order.
    """

    events = [(condition, True) for condition in sorted(active - previous)]
    if reassert:
        events += [(condition, True) for condition in sorted(active & previous)]
    events += [(condition, False) for condition in sorted(previous - active)]
    return events


def _platform_unavailable_error(error: Exception) -> bool:
    return type(error).__name__ in {"NotPoweredOnError", "KeepaliveMotorsOffError"}


class SpotSidecar:
    """Stateless bridge between the Zenoh bus and one Spot's BD SDK clients.

    Owns no goal state: velocity/manipulation goals arrive over the bus and are
    translated to SDK actuation, while Spot's live pose, velocity and faults are
    republished. Command state is guarded by self._lock; see _enqueue_actuation
    for the single-in-flight actuation invariant.
    """


    def __init__(
        self,
        robot,
        agent_id: str,
        mocap_body_id: int = 1,
        t_map_mocap=None,
        mocap_to_body_translation_m: tuple = (0.0, 0.0, 0.0),
        object_catalog_seeds: list | None = None,
        owns_object_state: bool = False,
        virtual_object_ids: set | None = None,
    ) -> None:
        self.agent_id = agent_id
        self.command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self.state_client = robot.ensure_client(RobotStateClient.default_service_name)

        self._active_fault_ids = set()
        self._fault_cycle = 0
        self._rejected_command_log = set()
        # Reentrant: on_command enqueues an actuation while already holding
        # this lock, and the actuation gate re-takes it internally (see
        # _enqueue_actuation and friends below).
        self._lock = threading.RLock()
        self._pending = None  # latest unhandled PlatformCommand (latest-wins)
        self._latest_cmd_stamp_ms = 0
        self._bus_cmd_id = None  # last-accepted velocity goal's command_id, for the goal-changed log line
        self._reported_velocity_dropped_for_arm = False
        self._reported_trajectory_dropped = False
        # Shared gate for mobility velocity and stop RPCs: at most one of
        # those requests is in flight at a time. Arm phases execute
        # sequentially on their own worker.
        self._actuation_inflight = False
        self._actuation_pending = None  # ("velocity", RobotCommand) | ("stop", None) | None
        self._motion_active = False  # a velocity has been sent and no confirmed stop has followed it yet
        self._unavailable_command_id = None
        self._last_state_published_ms = None
        self._state_future = None  # in-flight get_robot_state_async, harvested next step

        self._calibration_lock = threading.Lock()
        self._pending_calibration = None  # latest unconsumed FrameCalibration bytes
        self._t_map_vision = None  # None until calibrated
        self._t_vision_map = None
        self._reported_holding = False  # edge-trigger the "holding" log line

        self._t_map_mocap = t_map_mocap  # (rotation_rows, translation) or None
        self._mocap_body_id = mocap_body_id
        self._mocap_to_body_translation_m = tuple(mocap_to_body_translation_m)
        self._mocap_lock = threading.Lock()
        self._mocap_samples = collections.deque(maxlen=1024)  # (t_recv, x, y, z, yaw) in map
        self._vision_history = collections.deque(maxlen=256)  # (t, x, y, z, yaw) in vision
        self._cal_pub = None
        self._last_cal_wait_log = 0.0

        self._arm_thread = None
        self._arm_abort_requested = threading.Event()
        self._manipulation_results = queue.Queue()

        self._owns_object_state = owns_object_state
        self._object_catalog_seeds = object_catalog_seeds or []
        self._object_states = {}
        self._object_states_seeded = False
        self._virtual_object_ids = virtual_object_ids or set()

    @property
    def auto_calibrate(self) -> bool:
        return self._t_map_mocap is not None

    def on_control_request(self, sample) -> None:
        """Translate one generic JSON ControlRequest into the Spot executor."""
        try:
            request = json.loads(bytes(sample.payload))
            if request["agent_id"] != self.agent_id:
                return
            if request["approved_by"] != APPROVED_BY:
                return

            action_name, action = next(iter(request["action"].items()))
            if action_name in ("Hold", "Stop", "PlanarVelocity"):
                command = pb.PlatformCommand(
                    agent_id=request["agent_id"],
                    command_id=request["request_id"],
                    source=_control_source(request["source"]),
                    approved_by=request["approved_by"],
                    stamp_ms=request["stamp_ms"],
                    frame_id=action.get("frame_id", MAP_FRAME),
                    episode_id=request.get("safety_episode_id") or "",
                    command_authority=_control_source(request["source"]),
                )
                if action_name == "PlanarVelocity":
                    command.command_type = pb.COMMAND_TYPE_VELOCITY
                    command.target_position.CopyFrom(
                        _control_position(action["target"])
                    )
                    command.velocity.CopyFrom(
                        pb.SE2VelocitySetpoint(
                            forward_m_s=action["forward_m_s"],
                            left_m_s=action["left_m_s"],
                            yaw_rate_rad_s=action["yaw_rate_rad_s"],
                        )
                    )
                else:
                    command.command_type = (
                        pb.COMMAND_TYPE_STOP
                        if action_name == "Stop"
                        else pb.COMMAND_TYPE_HOLD
                    )
                    command.target_position.CopyFrom(
                        _control_position(action["target"])
                    )
                self._accept_platform_command(command)
                return

            if action_name in ("Pick", "Place"):
                target = action["target_pose"]
                command = pb.ManipulationCommand(
                    agent_id=request["agent_id"],
                    command_id=request["request_id"],
                    object_id=action["object_id"],
                    verb=(
                        pb.MANIPULATION_VERB_PICK_UP
                        if action_name == "Pick"
                        else pb.MANIPULATION_VERB_DROP_OFF
                    ),
                    approved_by=request["approved_by"],
                    target_pose=pb.PoseXYZYaw(
                        x=target["x"],
                        y=target["y"],
                        z=target["z"],
                        yaw_rad=target["yaw_rad"],
                    ),
                    stamp_ms=request["stamp_ms"],
                    frame_id=action["frame_id"],
                    grasp_profile=pb.MANIPULATION_GRASP_PROFILE_TOP_DOWN,
                )
                self._accept_manipulation_command(command)
                return

            print(f"sidecar: dropping unsupported control action {action_name!r}")
        except (KeyError, TypeError, ValueError, StopIteration) as error:
            print(f"sidecar: dropping malformed control request: {error}")

    def on_command(self, sample) -> None:
        """Handle an inbound PlatformCommand (zenoh subscriber thread).

        Rejected commands (auth gate) and stale ones (older stamp than the last
        seen) are dropped. A calibrated VELOCITY is dispatched immediately and
        non-blocking via _dispatch_velocity -- not parked for step() -- so the
        setpoint stream is decoupled from the 40 Hz loop; the single-in-flight
        actuation gate (see _enqueue_actuation) is what throttles it, not a
        fixed interval. Other command types are parked in _pending for the main
        loop, and HOLD/STOP additionally trigger an immediate reflex stop.
        """

        cmd = pb.PlatformCommand()
        cmd.ParseFromString(bytes(sample.payload))
        self._accept_platform_command(cmd)

    def _accept_platform_command(self, cmd) -> None:
        rejection = _command_rejection(cmd, self.agent_id)
        if rejection is not None:
            log_key = (cmd.agent_id, rejection.split("=")[0])
            if log_key not in self._rejected_command_log:
                self._rejected_command_log.add(log_key)
                print(
                    f"sidecar: dropping platform command: {rejection} "
                    "(further drops from this sender are silent)"
                )
            return

        with self._lock:
            if cmd.stamp_ms < self._latest_cmd_stamp_ms:
                return
            self._latest_cmd_stamp_ms = cmd.stamp_ms

            if cmd.command_type == pb.COMMAND_TYPE_VELOCITY:
                if self._t_map_vision is None:
                    return  # pre-calibration: the sidecar dispatches nothing
                if not _may_replace_pending(self._pending, cmd):
                    return
                self._pending = None  # latest-wins: supersedes any parked command
                self._dispatch_velocity(cmd)
                return
            self._pending = cmd  # processed in the main loop, no cross-thread RPC
            if cmd.command_type in (pb.COMMAND_TYPE_HOLD, pb.COMMAND_TYPE_STOP):
                self._reflex_stop(cmd.episode_id)

    def on_manipulation_command(self, sample) -> None:
        cmd = pb.ManipulationCommand()
        cmd.ParseFromString(bytes(sample.payload))
        self._accept_manipulation_command(cmd)

    def _accept_manipulation_command(self, cmd) -> None:
        rejection = _command_rejection(cmd, self.agent_id)
        if rejection is not None:
            log_key = (cmd.agent_id, "manipulation:" + rejection.split("=")[0])
            if log_key not in self._rejected_command_log:
                self._rejected_command_log.add(log_key)
                print(
                    f"sidecar: dropping manipulation command: {rejection} "
                    "(further drops from this sender are silent)"
                )
            return

        with self._lock:
            if self._arm_thread is not None and self._arm_thread.is_alive():
                print(f"sidecar: dropping manipulation command {cmd.command_id!r}: task already in flight")
                return
            if self._t_vision_map is None:
                print(f"sidecar: dropping manipulation command {cmd.command_id!r}: not calibrated yet")
                return
            is_virtual = cmd.object_id in self._virtual_object_ids
            self._arm_thread = threading.Thread(
                target=self._arm_worker,
                args=(cmd, self._t_vision_map, is_virtual),
                daemon=True
            )
            self._arm_thread.start()

    def on_calibration(self, sample) -> None:
        cal = pb.FrameCalibration()
        cal.ParseFromString(bytes(sample.payload))
        with self._calibration_lock:
            self._pending_calibration = cal

    def on_mocap(self, sample) -> None:

        frame = pb.MocapRigidBodyFrame()
        frame.ParseFromString(bytes(sample.payload))
        body = next((b for b in frame.bodies if b.id == self._mocap_body_id), None)
        if body is None:
            return
        p, q = body.position, body.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-6:
            return
        yaw = _mocap_yaw_from_quat(q.x, q.y, q.z, q.w)
        rows, trans = self._t_map_mocap
        mx, my, mz = (
            rows[i][0] * p.x + rows[i][1] * p.y + rows[i][2] * p.z + trans[i]
            for i in range(3)
        )
        with self._mocap_lock:
            self._mocap_samples.append((time.time(), mx, my, mz, yaw))

    def step(
        self,
        state_pub,
        constraint_pub,
        manipulation_status_pub,
        object_state_pub=None,
    ) -> None:
        """One 40 Hz (STATE_HZ) tick of the main bridge loop.

        Order matters: poll Spot state, publish faults, run/await calibration
        and hold until it locks, then publish agent_state, refresh owned object
        states, drain any command parked by on_command, and emit finished
        manipulation results.
        """
        tform, state = self._vision_tform_body()
        self._publish_fault(constraint_pub, state)
        if self.auto_calibrate:
            self._try_auto_calibrate(tform, state)
        else:
            self._try_lock_calibration(tform)

        if self._t_map_vision is None:
            if not self._reported_holding:
                print(
                    "sidecar: holding — "
                    + (
                        "auto-calibration waiting for a stationary mocap fix"
                        if self.auto_calibrate
                        else "no FrameCalibration for this agent_id yet"
                    )
                )
                self._reported_holding = True
            return

        self._publish_state(state_pub, tform, state)

        agent_position_map = agent_heading_yaw_rad = None
        if self._owns_object_state:
            agent_position_map = _mat4_apply_point(self._t_map_vision, (tform.x, tform.y, tform.z))
            agent_heading_yaw_rad = _map_heading_from_vision_pose(self._t_map_vision, tform)
            if not self._object_states_seeded:
                self._seed_object_states(object_state_pub)
            self._republish_held_object_states(
                object_state_pub, agent_position_map, agent_heading_yaw_rad
            )

        with self._lock:
            pending, self._pending = self._pending, None
        if pending is not None:
            self._handle_command(pending, tform)

        while not self._manipulation_results.empty():
            status, command = self._manipulation_results.get()
            manipulation_status_pub.put(status.SerializeToString())
            if self._owns_object_state and status.phase == pb.MANIPULATION_PHASE_SUCCEEDED:
                self._apply_manipulation_success(
                    object_state_pub, command, agent_position_map, agent_heading_yaw_rad
                )

    def _republish_held_object_states(self, pub, agent_position_map, agent_heading_yaw_rad) -> None:

        for object_id, state in self._object_states.items():
            if state["lifecycle"] == pb.OBJECT_LIFECYCLE_HELD and state["holder_agent_id"] == self.agent_id:
                state["pose"] = self._carry_pose_map(agent_position_map, agent_heading_yaw_rad)
                self._publish_object_state(pub, object_id, state, now_ms())


    def _block_for_feedback(self, cmd_id, check_func, timeout_sec):
        deadline = time.time() + timeout_sec
        last_feedback_error = None
        while time.time() < deadline:
            if self._arm_abort_requested.is_set():
                raise ArmSafetyStop("safety stop requested")
            try:
                response = self.command_client.robot_command_feedback(cmd_id)
            except Exception as error:
                last_feedback_error = error
                time.sleep(0.05)
                continue

            outcome = check_func(response)
            if outcome == "COMPLETE":
                return True
            if outcome.startswith("FAILED"):
                raise RuntimeError(outcome)
            time.sleep(0.05)
        if last_feedback_error is not None:
            raise RuntimeError(
                f"feedback timeout; last SDK feedback error: {last_feedback_error}"
            ) from last_feedback_error
        raise RuntimeError("feedback timeout")

    def _execute_arm_phase(self, phase_name, cmd, check_func):
        if self._arm_abort_requested.is_set():
            raise ArmSafetyStop("safety stop requested")
        if cmd == "software_pause":
            time.sleep(1.0)
            return

        print(f"sidecar: arm phase {phase_name!r} dispatched")
        try:
            cmd_id = self.command_client.robot_command(cmd)
        except Exception as error:
            raise RuntimeError(f"{phase_name} dispatch failed: {error}") from error

        try:
            self._block_for_feedback(cmd_id, check_func, ARM_PHASE_TIMEOUT_S)
        except ArmSafetyStop:
            raise
        except NoObjectDetected as error:
            raise NoObjectDetected(f"{phase_name}: {error}") from error
        except RuntimeError as error:
            raise RuntimeError(f"{phase_name}: {error}") from error
        print(f"sidecar: arm phase {phase_name!r} complete")

    def _arm_worker(self, command, t_vision_map, is_virtual):
        """Run one pick-up or drop-off arm sequence (dedicated daemon thread).

        Executes the fixed phase sequence for the verb. A non-safety phase
        failure after motion starts attempts recovery (gripper open, then stow).
        An active safety stop suppresses further arm commands, including
        recovery. The result is pushed to _manipulation_results for the main
        loop to publish. is_virtual relaxes grasp success (see _grasp_status):
        with no physical object present, the gripper reaching its closed goal
        counts as success.
        """
        verb_name = (
            "DROP_OFF"
            if command.verb == pb.MANIPULATION_VERB_DROP_OFF
            else "PICK_UP"
        )
        target = command.target_pose
        print(
            f"sidecar: arm task starting: {verb_name} "
            f"{command.object_id!r} (cmd {command.command_id})"
        )

        def _named_position_status(response):
            fb = response.feedback.synchronized_feedback.arm_command_feedback.named_arm_position_feedback
            status = arm_command_pb2.NamedArmPositionsCommand.Feedback
            if fb.status in (status.STATUS_COMPLETE, status.STATUS_STALLED_HOLDING_ITEM): return "COMPLETE"
            return "PENDING"

        def _gripper_status(response):
            fb = response.feedback.synchronized_feedback.gripper_command_feedback.claw_gripper_feedback
            status = gripper_command_pb2.ClawGripperCommand.Feedback
            if fb.status in (status.STATUS_AT_GOAL, status.STATUS_APPLYING_FORCE): return "COMPLETE"
            if fb.status == status.STATUS_UNKNOWN: return "FAILED: gripper status unknown"
            return "PENDING"

        def _grasp_status(response):
            fb = response.feedback.synchronized_feedback.gripper_command_feedback.claw_gripper_feedback
            status = gripper_command_pb2.ClawGripperCommand.Feedback
            if fb.status == status.STATUS_APPLYING_FORCE:
                print("sidecar: gripper detected contact and is applying force")
                return "COMPLETE"
            # Fully closed (AT_GOAL) means the jaws met nothing: a miss for a real
            # object, but the expected outcome for a virtual one (nothing to grip).
            if fb.status == status.STATUS_AT_GOAL:
                if is_virtual:
                    print("sidecar: virtual grasp reached the closed gripper target")
                    return "COMPLETE"
                raise NoObjectDetected(
                    "gripper reached fully closed without detecting object contact"
                )
            if fb.status == status.STATUS_UNKNOWN:
                return "FAILED: gripper status unknown"
            return "PENDING"

        success = False
        failure_reason = pb.MANIPULATION_FAILURE_REASON_UNSPECIFIED
        arm_motion_started = False

        try:
            robot_state = self.state_client.get_robot_state()
            t_vision_ground = get_a_tform_b(
                robot_state.kinematic_state.transforms_snapshot,
                VISION_FRAME_NAME,
                GROUND_PLANE_FRAME_NAME,
            )
            if t_vision_ground is None:
                raise Exception("GROUND_PLANE_UNAVAILABLE")

            def _move_command(phase_name, position):
                arm_command, requested_height = _grip_pose_command(
                    position,
                    target.yaw_rad,
                    t_vision_map,
                    t_vision_ground,
                )
                print(
                    f"sidecar: exact arm target {phase_name}: map "
                    f"({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}), "
                    f"grip height {requested_height:.3f} m above ground"
                )
                return arm_command

            exact_target = (target.x, target.y, target.z)
            _grip_pose_in_vision(
                exact_target,
                target.yaw_rad,
                t_vision_map,
                t_vision_ground,
            )
            arm_motion_started = True

            if command.verb == pb.MANIPULATION_VERB_DROP_OFF:
                place = exact_target
                hover = (target.x, target.y, target.z + HOVER_OFFSET_M)

                self._execute_arm_phase(
                    "PRE_PLACE",
                    _move_command("PRE_PLACE", hover),
                    _arm_cartesian_status,
                )
                self._execute_arm_phase(
                    "PLACE",
                    _move_command("PLACE", place),
                    _arm_cartesian_status,
                )
                self._execute_arm_phase(
                    "OPEN",
                    RobotCommandBuilder.claw_gripper_open_fraction_command(1.0),
                    _gripper_status,
                )
                self._execute_arm_phase(
                    "RETRACT",
                    _move_command("RETRACT", hover),
                    _arm_cartesian_status,
                )
                self._execute_arm_phase(
                    "CLOSE",
                    RobotCommandBuilder.claw_gripper_close_command(),
                    _gripper_status,
                )
                self._execute_arm_phase(
                    "STOW",
                    RobotCommandBuilder.arm_stow_command(),
                    _named_position_status,
                )
            else:
                grasp = exact_target
                pre_grasp = (target.x, target.y, target.z + HOVER_OFFSET_M)

                self._execute_arm_phase(
                    "UNSTOW",
                    RobotCommandBuilder.arm_ready_command(),
                    _named_position_status,
                )
                self._execute_arm_phase(
                    "OPEN",
                    RobotCommandBuilder.claw_gripper_open_fraction_command(1.0),
                    _gripper_status,
                )
                self._execute_arm_phase(
                    "PRE_GRASP",
                    _move_command("PRE_GRASP", pre_grasp),
                    _arm_cartesian_status,
                )
                self._execute_arm_phase(
                    "GRASP",
                    _move_command("GRASP", grasp),
                    _arm_cartesian_status,
                )
                self._execute_arm_phase(
                    "CLOSE",
                    RobotCommandBuilder.claw_gripper_close_command(),
                    _grasp_status,
                )
                self._execute_arm_phase("PAUSE", "software_pause", None)
                self._execute_arm_phase(
                    "CARRY",
                    RobotCommandBuilder.arm_carry_command(),
                    _named_position_status,
                )

            success = True
        except Exception as error:
            print(f"sidecar: arm task failed: {error}")
            if isinstance(error, ArmSafetyStop):
                failure_reason = pb.MANIPULATION_FAILURE_REASON_SAFETY_STOP
            elif isinstance(error, NoObjectDetected):
                failure_reason = (
                    pb.MANIPULATION_FAILURE_REASON_NO_OBJECT_DETECTED
                )
            elif isinstance(error, UnsafeGripHeight):
                failure_reason = pb.MANIPULATION_FAILURE_REASON_UNSAFE_GRIP_HEIGHT

            if isinstance(error, ArmSafetyStop):
                print(
                    "sidecar: arm recovery skipped while the safety stop "
                    "remains active"
                )
            elif arm_motion_started:
                try:
                    print("sidecar: running arm recovery (OPEN -> STOW)")
                    self._execute_arm_phase(
                        "RECOVERY_OPEN",
                        RobotCommandBuilder.claw_gripper_open_fraction_command(1.0),
                        _gripper_status,
                    )
                    self._execute_arm_phase(
                        "RECOVERY_STOW",
                        RobotCommandBuilder.arm_stow_command(),
                        _named_position_status,
                    )
                except Exception as recovery_error:
                    print(f"sidecar: arm recovery failed: {recovery_error}")
            else:
                print("sidecar: arm task rejected before motion; recovery not required")

        status = pb.ManipulationStatus(
            agent_id=command.agent_id,
            command_id=command.command_id,
            object_id=command.object_id,
            phase=pb.MANIPULATION_PHASE_SUCCEEDED if success else pb.MANIPULATION_PHASE_FAILED,
            failure_reason=failure_reason,
            stamp_ms=int(time.time() * 1000),
        )
        self._manipulation_results.put((status, command))

    def _try_lock_calibration(self, tform) -> None:
        """One-shot: lock map<->vision from an externally supplied FrameCalibration.

        Consumes a pending calibration addressed to this agent_id for the
        spot_body child frame (from a separate calibration terminal). It carries
        map<->body; composing with the current vision<->body pose yields the
        map<->vision registration held for the process lifetime.
        """
        if self._t_map_vision is not None:
            return
        with self._calibration_lock:
            cal, self._pending_calibration = self._pending_calibration, None
        if cal is None:
            return
        if cal.agent_id != self.agent_id or cal.child_frame != CHILD_FRAME_SPOT_BODY:
            return
        t_map_body = list(cal.t_parent_child)
        t_body_map = list(cal.t_child_parent)
        if (
            len(t_map_body) != 16
            or len(t_body_map) != 16
            or not all(math.isfinite(v) for v in t_map_body + t_body_map)
        ):
            print(f"sidecar: ignoring malformed FrameCalibration (agent {cal.agent_id})")
            return

        t_vision_body = _se3_pose_to_mat4(tform)
        t_body_vision = _se3_pose_to_mat4(tform.inverse())
        self._t_map_vision = _mat4_compose(t_map_body, t_body_vision)
        self._t_vision_map = _mat4_compose(t_vision_body, t_body_map)
        print(f"sidecar: locked map<->vision registration (agent {cal.agent_id})")

    def _try_auto_calibrate(self, tform, state) -> None:
        """One-shot: self-register map<->vision from a stationary mocap fix.

        Waits for a window where both the mocap stream and the robot's odometry
        are stationary (spreads under the CAL_* thresholds), then averages it.
        Builds map<->body as a yaw-only transform (Spot stands gravity-aligned,
        so roll/pitch are ignored) from the complete mocap position and mean
        yaw. Composing with the current vision<->body pose gives the
        registration, which is also published so other agents can adopt the
        same map frame.
        """

        if self._t_map_vision is not None:
            return
        now = time.time()
        self._vision_history.append((now, tform.x, tform.y, tform.z, tform.rot.to_yaw()))
        with self._mocap_lock:
            samples = list(self._mocap_samples)

        reason = None
        window = [s for s in samples if now - s[0] <= CAL_WINDOW_S]
        if not samples:
            reason = f"no mocap frames for body id {self._mocap_body_id} yet"
        elif now - samples[-1][0] > CAL_MAX_STALE_S:
            reason = f"mocap stream stale ({now - samples[-1][0]:.1f} s)"
        elif len(window) < CAL_MIN_SAMPLES or window[-1][0] - window[0][0] < CAL_WINDOW_S * 0.9:
            reason = f"collecting stationary window ({len(window)} frames)"
        else:
            mx, my, mz, myaw, m_pos_spread, m_yaw_spread = _pose_window_stats(
                [s[1:] for s in window]
            )
            vision_window = [v[1:] for v in self._vision_history if now - v[0] <= CAL_WINDOW_S]
            _, _, v_mz, _, v_pos_spread, v_yaw_spread = _pose_window_stats(vision_window)
            if m_pos_spread > CAL_MAX_POS_SPREAD_M or m_yaw_spread > CAL_MAX_YAW_SPREAD_RAD:
                reason = (
                    f"mocap not stationary (spread {m_pos_spread * 100:.1f} cm / "
                    f"{math.degrees(m_yaw_spread):.2f}°)"
                )
            elif v_pos_spread > CAL_MAX_POS_SPREAD_M or v_yaw_spread > CAL_MAX_YAW_SPREAD_RAD:
                reason = (
                    f"robot moving in odometry (spread {v_pos_spread * 100:.1f} cm / "
                    f"{math.degrees(v_yaw_spread):.2f}°)"
                )
            else:
                mocap_origin_map = (mx, my, mz)
                body_position_map = _spot_body_position_map(
                    mocap_origin_map,
                    myaw,
                    self._mocap_to_body_translation_m,
                )
                (
                    t_map_body,
                    t_body_map,
                    self._t_map_vision,
                    self._t_vision_map,
                ) = _map_vision_registration(body_position_map, myaw, tform)
                print(
                    f"sidecar: auto-calibrated map<->vision — Motive origin at map "
                    f"({mx:.3f}, {my:.3f}, {mz:.3f}); Spot BODY at map "
                    f"({body_position_map[0]:.3f}, {body_position_map[1]:.3f}, "
                    f"{body_position_map[2]:.3f}) using BODY-axis offset "
                    f"({self._mocap_to_body_translation_m[0]:.3f}, "
                    f"{self._mocap_to_body_translation_m[1]:.3f}, "
                    f"{self._mocap_to_body_translation_m[2]:.3f}); "
                    f"yaw {math.degrees(myaw):.1f}°, "
                    f"residuals {m_pos_spread * 100:.2f} cm / "
                    f"{math.degrees(m_yaw_spread):.3f}° over {len(window)} frames"
                )
                self._log_floor_diagnostic(state, v_mz)
                self._publish_calibration(t_map_body, t_body_map)
                return

        if now - self._last_cal_wait_log >= CAL_WAIT_LOG_PERIOD_S:
            print(f"sidecar: auto-calibrate waiting — {reason}")
            self._last_cal_wait_log = now

    def _log_floor_diagnostic(self, state, body_z: float) -> None:

        snapshot = state.kinematic_state.transforms_snapshot
        foot_heights_vision = {}
        for name in FOOT_FRAME_NAMES:
            foot_tform_vision = get_a_tform_b(snapshot, VISION_FRAME_NAME, name)
            if foot_tform_vision is not None:
                foot_heights_vision[name] = foot_tform_vision.z
        if not foot_heights_vision:
            print("sidecar: floor diagnostic — no foot frames found in this state's snapshot")
            return
        mean_foot_z = sum(foot_heights_vision.values()) / len(foot_heights_vision)
        detail = ", ".join(f"{name}={z:.3f}" for name, z in foot_heights_vision.items())
        print(
            f"sidecar: floor diagnostic — foot heights in vision frame: {detail} "
            f"(mean {mean_foot_z:.3f}); body height {body_z:.3f}; "
            f"body-to-floor clearance {body_z - mean_foot_z:.3f} m"
        )

    def _publish_calibration(self, t_map_body: list, t_body_map: list) -> None:
        """Publish the locked map<->vision registration, both directions.

        Both `t_parent_child` and `t_child_parent` are sent so consumers never
        need to invert the transform themselves.
        """
        if self._cal_pub is None:
            return
        msg = pb.FrameCalibration(
            agent_id=self.agent_id,
            parent_frame=MAP_FRAME,
            child_frame=CHILD_FRAME_SPOT_BODY,
            t_parent_child=t_map_body,
            t_child_parent=t_body_map,
            stamp_ms=now_ms(),
        )
        self._cal_pub.put(msg.SerializeToString())

    def _handle_command(self, cmd, tform) -> None:
        """Dispatch a command parked in _pending, from the main loop.

        Non-"map" frames are dropped; VELOCITY re-dispatches through
        _dispatch_velocity (a backstop for the on_command fast path), TRAJECTORY
        is rejected once (position control moved onboard), and HOLD/STOP stop the
        robot.
        """
        if cmd.frame_id != MAP_FRAME:
            print(f"sidecar: dropping command with frame_id={cmd.frame_id!r} (want \"map\")")
            return

        if cmd.command_type == pb.COMMAND_TYPE_VELOCITY:
            self._dispatch_velocity(cmd)
        elif cmd.command_type == pb.COMMAND_TYPE_TRAJECTORY:
            if not self._reported_trajectory_dropped:
                self._reported_trajectory_dropped = True
                print(
                    "sidecar: dropping trajectory command (position controller "
                    "replaced by velocity controller; misconfigured producer?)"
                )
        else:  # COMMAND_TYPE_HOLD or COMMAND_TYPE_STOP
            self._stop()

    def _dispatch_velocity(self, cmd) -> None:
        """Build and enqueue a Spot velocity command from a bus setpoint.

        Requires frame_id == "map" and no manipulation task in flight. The
        setpoint (forward/left/yaw-rate) becomes a synchro_velocity_command in
        the gravity-aligned body frame carrying _VELOCITY_LIMIT_PARAMS, then goes
        through the actuation gate rather than being sent directly. The end-time
        keepalive (VELOCITY_CMD_EXPIRATION_S) is applied per send in
        _send_actuation.
        """

        if cmd.frame_id != MAP_FRAME:
            print(f"sidecar: dropping command with frame_id={cmd.frame_id!r} (want \"map\")")
            return

        if self._arm_thread is not None and self._arm_thread.is_alive():
            if not self._reported_velocity_dropped_for_arm:
                self._reported_velocity_dropped_for_arm = True
                print(f"sidecar: dropping velocity command {cmd.command_id!r}: a manipulation task is in flight")
            return
        self._reported_velocity_dropped_for_arm = False
        with self._lock:
            if self._unavailable_command_id == cmd.command_id:
                return
            self._unavailable_command_id = None

        command = RobotCommandBuilder.synchro_velocity_command(
            v_x=cmd.velocity.forward_m_s,
            v_y=cmd.velocity.left_m_s,
            v_rot=cmd.velocity.yaw_rate_rad_s,
            params=_VELOCITY_LIMIT_PARAMS,
            frame_name=GRAV_ALIGNED_BODY_FRAME_NAME,
        )

        if cmd.command_id != self._bus_cmd_id:
            tp = cmd.target_position
            print(f"sidecar: velocity tracking goal map({tp.x:.2f}, {tp.y:.2f}) (cmd {cmd.command_id})")
            self._bus_cmd_id = cmd.command_id

        self._enqueue_actuation(("velocity", command))

    def _reflex_stop(self, episode_id: str) -> None:

        if episode_id:
            self._arm_abort_requested.set()
        else:
            self._arm_abort_requested.clear()

        with self._lock:
            if not self._motion_active:
                return  # nothing believed to be moving: no stop RPC needed
        self._enqueue_actuation(("stop", None))

    def _enqueue_actuation(self, action) -> None:
        """Queue `action` — `("velocity", RobotCommand)` or `("stop", None)` —
        as the next thing to send over `command_client`, dispatching it right
        away if nothing is currently in flight.

        This is the single gate for velocity dispatch, the immediate reflex
        stop, and the main-loop stop retry. Each completion self-triggers the
        next queued mobility action (`_on_actuation_done`) — no timer or
        fixed-rate dispatch loop. A stop always overwrites a pending velocity,
        and a velocity never overwrites a pending stop.
        """
        to_send = None
        with self._lock:
            kind = action[0]
            if kind == "stop" or self._actuation_pending is None or self._actuation_pending[0] != "stop":
                self._actuation_pending = action
            if not self._actuation_inflight and self._actuation_pending is not None:
                to_send = self._actuation_pending
                self._actuation_pending = None
                self._actuation_inflight = True
        if to_send is not None:
            self._send_actuation(to_send)

    def _advance_actuation(self) -> None:
        to_send = None
        with self._lock:
            if not self._actuation_inflight and self._actuation_pending is not None:
                to_send = self._actuation_pending
                self._actuation_pending = None
                self._actuation_inflight = True
        if to_send is not None:
            self._send_actuation(to_send)

    def _record_actuation_failure(self, error: Exception) -> None:
        if not _platform_unavailable_error(error):
            print(f"sidecar: command rejected: {type(error).__name__}: {error}")
            return

        with self._lock:
            command_id = self._bus_cmd_id or "<no-active-command>"
            newly_unavailable = self._unavailable_command_id != command_id
            self._unavailable_command_id = command_id
            self._actuation_pending = None
            self._pending = None
            self._motion_active = False
        if newly_unavailable:
            print(
                "sidecar: platform actuation unavailable: "
                f"{type(error).__name__}: {error}; "
                "suppressing repeated dispatches for this command"
            )

    def _send_actuation(self, action) -> None:
        kind, payload = action
        try:
            if kind == "velocity":
                future = self.command_client.robot_command_async(
                    payload, end_time_secs=time.time() + VELOCITY_CMD_EXPIRATION_S
                )
            else:
                future = self.command_client.robot_command_async(RobotCommandBuilder.stop_command())
        except Exception as exc:  # noqa: BLE001 - report; the next enqueue fires the next RPC
            self._record_actuation_failure(exc)
            with self._lock:
                self._actuation_inflight = False
            self._advance_actuation()
            return
        if kind == "velocity":
            with self._lock:
                self._motion_active = True
        future.add_done_callback(lambda fut, k=kind: self._on_actuation_done(k, fut))

    def _on_actuation_done(self, kind, fut) -> None:
        exc = fut.exception()
        if exc is not None and type(exc).__name__ != "ExpiredError":
            self._record_actuation_failure(exc)
        with self._lock:
            if kind == "stop" and exc is None:
                self._motion_active = False
            self._actuation_inflight = False
        self._advance_actuation()

    def _stop(self) -> None:
        self._reflex_stop("")

    def _vision_tform_body(self):
        # Pipelined state poll: harvest the RPC issued last step and start the
        # next one, so the 40 Hz cadence overlaps the RPC instead of blocking
        # on it. Worst case (RTT > one period) degrades to the old synchronous
        # wait; a failed harvest raises into step()'s loop-error handler while
        # the fresh in-flight request self-heals the pipeline next step.
        harvest = self._state_future
        self._state_future = self.state_client.get_robot_state_async()
        state = (harvest or self._state_future).result()
        snapshot = state.kinematic_state.transforms_snapshot
        return get_a_tform_b(snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME), state

    def _publish_state(self, pub, tform, state) -> None:
        """Publish AgentStateMsg: map-frame pose plus live velocity and yaw-rate.

        Position and heading come from the vision pose mapped through
        _t_map_vision. Linear velocity is velocity_of_body_in_vision rotated
        (rotation only) into map. yaw_rate is taken directly as angular.z: vision
        and map share the gravity +Z axis (calibration is yaw-only about Z), so
        the body's yaw rate needs no reframing.
        """
        mx, my, mz = _mat4_apply_point(self._t_map_vision, (tform.x, tform.y, tform.z))
        heading_yaw_rad = _map_heading_from_vision_pose(self._t_map_vision, tform)
        v = state.kinematic_state.velocity_of_body_in_vision
        vx, vy, vz = _mat4_apply_vector(self._t_map_vision, (v.linear.x, v.linear.y, v.linear.z))
        stamp_ms = now_ms()
        msg = pb.AgentStateMsg(
            agent_id=self.agent_id,
            source=pb.SOURCE_SPOT_ADAPTER,
            state_authority=pb.STATE_AUTHORITY_REAL,
            position=pb.Vec3(x=mx, y=my, z=mz),
            stamp_ms=stamp_ms,
            frame_id=MAP_FRAME,
            heading_yaw_rad=heading_yaw_rad,
            velocity=pb.Vec3(x=vx, y=vy, z=vz),
            yaw_rate_rad_s=v.angular.z,
        )
        pub.put(msg.SerializeToString())
        self._last_state_published_ms = stamp_ms

    def _carry_pose_map(self, agent_position_map: tuple, agent_heading_yaw_rad: float) -> tuple:
        """Dead-reckoned (x, y, z, yaw) of the object a carrier is holding.

        Placed half a body length ahead of the carrier along its heading, at
        fixed body height -- held objects have no independent tracking.
        """

        forward = SPOT_BODY_LENGTH_M * 0.5
        x, y, z = agent_position_map
        return (
            x + forward * math.cos(agent_heading_yaw_rad),
            y + forward * math.sin(agent_heading_yaw_rad),
            SPOT_BODY_HEIGHT_M,
            agent_heading_yaw_rad,
        )

    def _publish_object_state(self, pub, object_id: str, state: dict, stamp_ms: int) -> None:
        x, y, z, yaw = state["pose"]
        msg = pb.ObjectState(
            object_id=object_id,
            lifecycle=state["lifecycle"],
            holder_agent_id=state["holder_agent_id"],
            pose=pb.PoseXYZYaw(x=x, y=y, z=z, yaw_rad=yaw),
            pose_valid=True,
            stamp_ms=stamp_ms,
        )
        pub.put(msg.SerializeToString())

    def _seed_object_states(self, pub) -> None:

        stamp_ms = now_ms()
        for object_id, x, y, z, yaw in self._object_catalog_seeds:
            state = {
                "lifecycle": pb.OBJECT_LIFECYCLE_FREE,
                "holder_agent_id": "",
                "pose": (x, y, z, yaw),
            }
            self._object_states[object_id] = state
            self._publish_object_state(pub, object_id, state, stamp_ms)
        self._object_states_seeded = True

    def _apply_manipulation_success(
        self, pub, command, agent_position_map: tuple, agent_heading_yaw_rad: float
    ) -> None:

        if command.verb == pb.MANIPULATION_VERB_DROP_OFF:
            target = command.target_pose
            state = {
                "lifecycle": pb.OBJECT_LIFECYCLE_FREE,
                "holder_agent_id": "",
                "pose": (target.x, target.y, target.z, target.yaw_rad),
            }
        else:
            state = {
                "lifecycle": pb.OBJECT_LIFECYCLE_HELD,
                "holder_agent_id": self.agent_id,
                "pose": self._carry_pose_map(agent_position_map, agent_heading_yaw_rad),
            }
        self._object_states[command.object_id] = state
        self._publish_object_state(pub, command.object_id, state, now_ms())

    def _publish_fault(self, pub, state) -> None:
        """Level-publish active robot faults as ConstraintEvents.

        Lifecycle per condition: onset publishes SEVERITY_HARD_STOP once; while
        still active it is re-asserted every FAULT_REASSERT_CYCLES ticks (so a
        late subscriber still learns of it); recovery publishes a single
        SEVERITY_NONE clear. Active conditions are level-triggered off
        _fault_condition_ids.
        """

        active = _fault_condition_ids(state)
        self._fault_cycle += 1
        reassert = self._fault_cycle % FAULT_REASSERT_CYCLES == 0

        for condition_id, is_hard_stop in _fault_transitions(
            active, self._active_fault_ids, reassert
        ):
            msg = pb.ConstraintEvent(
                agent_id=self.agent_id,
                source=pb.SOURCE_SPOT_ADAPTER,
                collision_authority=pb.COLLISION_AUTHORITY_REAL_SENSORS,
                constraint_type=pb.CONSTRAINT_TYPE_ROBOT_FAULT,
                severity=pb.SEVERITY_HARD_STOP if is_hard_stop else pb.SEVERITY_NONE,
                object_id=condition_id,
                stamp_ms=now_ms(),
            )
            pub.put(msg.SerializeToString())

        if active != self._active_fault_ids:
            print(f"sidecar: fault conditions now {sorted(active) or 'clear'}")
        self._active_fault_ids = active


# The launcher runs this script from an extracted temp directory (see
# main.rs's `run_sidecar`), so a path relative to `__file__` would land in
# that temp directory, not the repo's `logs/` next to the operator UI's own
# `logs/operator_ui_*.md`. The launcher passes the real directory explicitly;
# a bare `__file__`-relative fallback only matters for running this script
# directly (matching `--frames-config`/`--object-catalog`'s own env-first,
# `__file__`-relative-fallback pattern above).
LOG_DIR = os.environ.get(
    "DCA_SIDECAR_LOG_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs"),
)


class _TimestampedTee:
    """Duplicates writes to the console and a timestamped log file.

    Lets every existing `print()` in this module double as durable,
    post-mortem logging without threading a logger through the file. The
    operator UI keeps its own separate transcript (`logs/operator_ui_*.md`,
    its command echoes and state-derived status lines) — this is the
    sidecar process's own stdout/stderr, for diagnosing what the bridge to
    Spot actually did.
    """

    def __init__(self, stream, log_file) -> None:
        self._stream = stream
        self._log_file = log_file
        self._at_line_start = True

    def write(self, text: str) -> None:
        self._stream.write(text)
        for chunk in text.splitlines(keepends=True):
            if self._at_line_start and chunk.strip():
                stamp = f"{time.strftime('%Y-%m-%d %H:%M:%S')}.{int(time.time() * 1000) % 1000:03d}"
                self._log_file.write(f"[{stamp}] ")
            self._log_file.write(chunk)
            self._at_line_start = chunk.endswith("\n")
        self._log_file.flush()

    def flush(self) -> None:
        self._stream.flush()
        self._log_file.flush()


def _start_console_logging(agent_id: str) -> str:
    os.makedirs(LOG_DIR, exist_ok=True)
    run_id = os.environ.get("DCA_RUN_ID") or time.strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(LOG_DIR, f"sidecar_{agent_id}_{run_id}.log")
    log_file = open(log_path, "a", encoding="utf-8")
    sys.stdout = _TimestampedTee(sys.stdout, log_file)
    sys.stderr = sys.stdout
    return log_path


def _lease_owner_name(owner) -> str | None:
    client_name = getattr(owner, "client_name", "").strip()
    user_name = getattr(owner, "user_name", "").strip()

    if client_name and user_name:
        return f"{client_name} (user {user_name})"
    return client_name or user_name or None


def _lease_owner_label(error: ResourceAlreadyClaimedError) -> str | None:
    response = getattr(error, "response", None)
    return _lease_owner_name(getattr(response, "lease_owner", None))


def _lease_keepalive_failure_detail(error: Exception) -> str:
    response = getattr(error, "response", None)
    result = getattr(response, "lease_use_result", None)
    status = getattr(result, "status", None)
    owner = _lease_owner_name(getattr(result, "owner", None))

    details = [type(error).__name__]
    if status is not None:
        status_name = getattr(getattr(result, "Status", None), "Name", None)
        if callable(status_name):
            try:
                status = status_name(status)
            except ValueError:
                pass
        details.append(f"status={status}")
    if owner:
        details.append(f"current owner={owner}")
    return ", ".join(details)


def _report_lease_keepalive_failure(error: Exception) -> None:
    print(
        "sidecar: lease keepalive check failed; SDK will retry: "
        f"{_lease_keepalive_failure_detail(error)}"
    )


def _acquire_body_lease(lease_client):
    try:
        return LeaseKeepAlive(
            lease_client,
            must_acquire=True,
            return_at_exit=True,
            on_failure_callback=_report_lease_keepalive_failure,
            # The callback above replaces the SDK's owner-less warning without
            # hiding the failure or changing its retry behavior.
            warnings=False,
        )
    except ResourceAlreadyClaimedError as error:
        owner = _lease_owner_label(error)
        owner_detail = f" Lease owner: {owner}." if owner else ""
        raise RuntimeError(
            "Spot body lease is already held by another controller."
            f"{owner_detail} Keep the controller connected for E-stop, but set "
            "Robot Body Control to No Control before retrying. "
            "DCA did not take the lease."
        ) from error


def _power_on(robot) -> None:
    try:
        robot.power_on(timeout_sec=20)
    except KeepaliveMotorsOffError as error:
        raise RuntimeError(
            "Spot motor power is blocked by an active Keepalive safety policy. "
            "Stop or clear the owning tablet, mission, AutoReturn, or SDK client, "
            "then retry. DCA did not remove the policy."
        ) from error


def _safe_power_off(robot) -> None:
    try:
        if robot.is_powered_on():
            print("sidecar: attempting safe power off...")
            robot.power_off(cut_immediately=False, timeout_sec=20)
            print("sidecar: robot safely powered off.")
    except Exception as error:
        print(f"sidecar: error during safe power off: {error}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Spot <-> decentralized_autonomy bus sidecar")
    parser.add_argument("hostname", nargs="?", help="Spot host/IP, e.g. 192.168.55.21")
    parser.add_argument("--runtime-self-test", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--agent-id", default="agent_01")
    parser.add_argument("--auto-power-on", action="store_true", help="power motors on at startup")
    parser.add_argument("--auto-stand", action="store_true", help="stand at startup (implies power-on)")
    parser.add_argument(
        "--auto-calibrate", action="store_true",
        help="self-register map<->vision from mocap_rigid_body after standing "
        "(replaces the separate calibration terminal)",
    )
    parser.add_argument(
        "--mocap-body-id", type=int, default=1,
        help="Motive streaming ID of Spot's rigid body (with --auto-calibrate)",
    )
    parser.add_argument(
        "--mocap-body-to-spot-body",
        type=float,
        nargs=3,
        metavar=("FORWARD_M", "LEFT_M", "UP_M"),
        default=(0.0, 0.0, 0.0),
        help="translation from the Motive rigid-body origin to Spot BODY, "
        "expressed in Spot BODY forward/left/up axes",
    )
    parser.add_argument(
        "--frames-config",
        default=os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "config", "frames.json"
        ),
        help="frames.json providing mocap_lab_to_map (with --auto-calibrate)",
    )
    parser.add_argument(
        "--owns-object-state", action="store_true",
        help="real mode only: this sidecar is the sole authority for the "
        "virtual object catalog/lifecycle (seeds ObjectState FREE at boot, publishes "
        "attach/detach on a completed manipulation task). Real mode runs no "
        "environment_sim process (see the module docstring's \"no virtual walls\" design), "
        "so nothing else would publish ObjectState. Do not pass this in hybrid mode — "
        "the mirror role already owns object-state truth there and would race this sidecar.",
    )
    parser.add_argument(
        "--object-catalog",
        default=os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "config", "objects.json"
        ),
        help="config/objects.json providing the virtual object catalog (with --owns-object-state)",
    )
    args = parser.parse_args()

    if args.runtime_self_test:
        bosdyn.client.create_standard_sdk("dca_spot_runtime_self_test")
        print("spot-runtime: self-test passed")
        return
    if args.hostname is None:
        parser.error("hostname is required")

    log_path = _start_console_logging(args.agent_id)
    print(f"sidecar: logging console output to {log_path}")
    shutdown_requested = _start_shutdown_listener()

    t_map_mocap = _load_t_map_mocap(args.frames_config) if args.auto_calibrate else None

    virtual_object_seeds = _load_object_catalog_seeds(args.object_catalog)
    virtual_object_ids = {s[0] for s in virtual_object_seeds}
    object_catalog_seeds = virtual_object_seeds if args.owns_object_state else []

    sdk = bosdyn.client.create_standard_sdk("onboard_agent_sidecar")
    robot = sdk.create_robot(args.hostname)
    bosdyn.client.util.authenticate(robot)  # reads BOSDYN_CLIENT_USERNAME/PASSWORD
    robot.time_sync.wait_for_sync()

    while robot.is_estopped():
        print("sidecar: waiting for e-stop to be released (acquire it on the tablet)...")
        if shutdown_requested.wait(1.0):
            print("sidecar: shutdown requested.")
            return

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    lease_keepalive = _acquire_body_lease(lease_client)
    try:
        with lease_keepalive:
            try:
                sidecar = SpotSidecar(
                    robot,
                    args.agent_id,
                    mocap_body_id=args.mocap_body_id,
                    t_map_mocap=t_map_mocap,
                    mocap_to_body_translation_m=tuple(
                        args.mocap_body_to_spot_body
                    ),
                    object_catalog_seeds=object_catalog_seeds,
                    owns_object_state=args.owns_object_state,
                    virtual_object_ids=virtual_object_ids,
                )

                if args.auto_power_on or args.auto_stand:
                    print("sidecar: powering motors on...")
                    _power_on(robot)
                if args.auto_stand:
                    print("sidecar: standing...")
                    blocking_stand(sidecar.command_client, timeout_sec=10)

                with _open_zenoh_session() as session:
                    # Liveliness token: withdrawn automatically when this
                    # process/session dies, letting the onboard agent detect
                    # death ahead of the state-staleness fallback.
                    liveliness_token = session.liveliness().declare_token(
                        topic(f"liveliness/agent_state/{sidecar.agent_id}")
                    )
                    state_pub = session.declare_publisher(
                        topic("agent_state"),
                        congestion_control=zenoh.CongestionControl.DROP,
                    )
                    constraint_pub = session.declare_publisher(
                        topic("constraint_event"),
                        congestion_control=zenoh.CongestionControl.BLOCK,
                        priority=zenoh.Priority.REAL_TIME,
                    )
                    manipulation_status_pub = session.declare_publisher(
                        topic("manipulation_status"),
                        congestion_control=zenoh.CongestionControl.DROP,
                    )
                    object_state_pub = (
                        session.declare_publisher(
                            topic("object_state"),
                            congestion_control=zenoh.CongestionControl.DROP,
                        )
                        if args.owns_object_state
                        else None
                    )
                    session.declare_subscriber(
                        topic("control_request"),
                        sidecar.on_control_request,
                    )
                    if args.auto_calibrate:
                        sidecar._cal_pub = session.declare_publisher(
                            topic("frame_calibration"),
                            congestion_control=zenoh.CongestionControl.DROP,
                        )
                        session.declare_subscriber(
                            topic("mocap_rigid_body"),
                            sidecar.on_mocap,
                        )
                        print(
                            f"sidecar: auto-calibrate on (mocap body {args.mocap_body_id}) — "
                            "waiting for a stationary fix"
                        )
                    else:
                        session.declare_subscriber(
                            topic("frame_calibration"),
                            sidecar.on_calibration,
                        )

                    print("sidecar: bridging Spot <-> bus (frame=vision). Ctrl-C to stop.")
                    period = 1.0 / STATE_HZ
                    next_deadline = time.time() + period
                    while not shutdown_requested.is_set():
                        try:
                            sidecar.step(
                                state_pub,
                                constraint_pub,
                                manipulation_status_pub,
                                object_state_pub,
                            )
                        except Exception as error:
                            print(
                                f"sidecar: loop error: {type(error).__name__}: {error}"
                            )

                        now = time.time()
                        if now > next_deadline:
                            next_deadline = now
                        else:
                            time.sleep(next_deadline - now)
                        next_deadline += period
                    print("\nsidecar: shutdown requested.")
            except KeyboardInterrupt:
                print("\nsidecar: shutdown requested.")
            finally:
                _safe_power_off(robot)
    finally:
        print("sidecar: lease keepalive stopped; lease return requested.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        import traceback
        print("\n" + "="*60)
        print(f"FATAL ERROR: {exc}")
        print("="*60)
        traceback.print_exc()
        print("="*60)
        sys.stdout.flush()
        sys.exit(1)
