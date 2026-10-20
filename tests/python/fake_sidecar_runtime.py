"""Hermetic import/runtime support for the Spot-side Python checks.

The production modules import protobuf, Zenoh, NumPy, and the Boston Dynamics
SDK at module load time.  This support module provides the narrow interfaces
the production code actually exercises so the suite can run with only the
Python standard library.  No fake opens a socket or creates an SDK session.
"""

from __future__ import annotations

import copy
import importlib.util
import math
import pickle
import sys
import types
from collections import deque
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SIDECAR_ROOT = REPO_ROOT / "platforms" / "spot"
IDENTITY_4X4 = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
]


def namespace(**values):
    return types.SimpleNamespace(**values)


class Struct:
    """Small protobuf/SDK-shaped record accepting keyword fields."""

    def __init__(self, **values):
        self.__dict__.update(values)

    def CopyFrom(self, other):
        self.__dict__ = copy.deepcopy(other.__dict__)


class FakeZenohConfig:
    def __init__(self):
        self.values = {}

    def insert_json5(self, key, value):
        self.values[key] = value


class ProtoMessage(Struct):
    defaults: dict[str, object] = {}

    def __init__(self, **values):
        fields = {}
        for name, default in self.defaults.items():
            fields[name] = default() if callable(default) else copy.deepcopy(default)
        fields.update(values)
        super().__init__(**fields)

    def SerializeToString(self) -> bytes:
        return pickle.dumps(self, protocol=pickle.HIGHEST_PROTOCOL)

    def ParseFromString(self, payload: bytes) -> None:
        decoded = pickle.loads(bytes(payload))
        if not isinstance(decoded, self.__class__):
            raise ValueError(f"expected {self.__class__.__name__}, got {type(decoded).__name__}")
        self.__dict__ = copy.deepcopy(decoded.__dict__)

    @classmethod
    def FromString(cls, payload: bytes):
        message = cls()
        message.ParseFromString(payload)
        return message


class Vec3(ProtoMessage):
    defaults = {"x": 0.0, "y": 0.0, "z": 0.0}


class QuatMessage(ProtoMessage):
    defaults = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}


class PoseXYZYaw(ProtoMessage):
    defaults = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_rad": 0.0}


class SE2VelocitySetpoint(ProtoMessage):
    defaults = {"forward_m_s": 0.0, "left_m_s": 0.0, "yaw_rate_rad_s": 0.0}


class PlatformCommand(ProtoMessage):
    defaults = {
        "agent_id": "",
        "command_id": "",
        "command_type": 0,
        "source": 0,
        "approved_by": "",
        "target_position": Vec3,
        "stamp_ms": 0,
        "frame_id": "",
        "episode_id": "",
        "velocity": SE2VelocitySetpoint,
    }


class ManipulationCommand(ProtoMessage):
    defaults = {
        "agent_id": "",
        "command_id": "",
        "object_id": "",
        "verb": 0,
        "approved_by": "",
        "target_pose": PoseXYZYaw,
        "stamp_ms": 0,
        "frame_id": "",
        "grasp_profile": 0,
    }


class ManipulationStatus(ProtoMessage):
    defaults = {
        "agent_id": "",
        "command_id": "",
        "object_id": "",
        "phase": 0,
        "failure_reason": 0,
        "stamp_ms": 0,
    }


class CommandStatus(ProtoMessage):
    defaults = {"agent_id": "", "command_id": "", "status": 0, "stamp_ms": 0}


class ConstraintEvent(ProtoMessage):
    defaults = {
        "agent_id": "",
        "source": 0,
        "collision_authority": 0,
        "constraint_type": 0,
        "severity": 0,
        "object_id": "",
        "stamp_ms": 0,
        "episode_id": "",
        "trip_state_stamp_ms": 0,
        "detail": "",
    }


class AgentStateMsg(ProtoMessage):
    defaults = {
        "agent_id": "",
        "source": 0,
        "state_authority": 0,
        "position": Vec3,
        "stamp_ms": 0,
        "frame_id": "",
        "heading_yaw_rad": 0.0,
        "velocity": Vec3,
        "yaw_rate_rad_s": 0.0,
    }


class ObjectState(ProtoMessage):
    defaults = {
        "object_id": "",
        "lifecycle": 0,
        "holder_agent_id": "",
        "pose": PoseXYZYaw,
        "pose_valid": False,
        "stamp_ms": 0,
    }


class RigidBodyPose(ProtoMessage):
    defaults = {"id": 0, "position": Vec3, "orientation": QuatMessage}


class MocapRigidBodyFrame(ProtoMessage):
    defaults = {"bodies": list, "stamp_ms": 0, "frame_id": ""}


class FrameCalibration(ProtoMessage):
    defaults = {
        "agent_id": "",
        "parent_frame": "",
        "child_frame": "",
        "t_parent_child": list,
        "t_child_parent": list,
        "stamp_ms": 0,
    }


def build_fake_protobuf_module():
    module = types.ModuleType("core_msgs_pb2")
    constants = {
        "COMMAND_TYPE_TRAJECTORY": 0,
        "COMMAND_TYPE_HOLD": 1,
        "COMMAND_TYPE_STOP": 2,
        "COMMAND_TYPE_VELOCITY": 3,
        "COMMAND_SOURCE_AUTONOMY": 0,
        "COMMAND_SOURCE_HUMAN_OPERATOR": 1,
        "COMMAND_SOURCE_NONE": 255,
        "SOURCE_SPOT_ADAPTER": 1,
        "STATE_AUTHORITY_REAL": 2,
        "COLLISION_AUTHORITY_REAL_SENSORS": 2,
        "CONSTRAINT_TYPE_ROBOT_FAULT": 1,
        "CONSTRAINT_TYPE_GOAL_BLOCKED": 3,
        "SEVERITY_NONE": 0,
        "SEVERITY_HARD_STOP": 1,
        "COMMAND_STATUS_CODE_SUCCEEDED": 0,
        "MANIPULATION_VERB_UNSPECIFIED": 0,
        "MANIPULATION_VERB_PICK_UP": 1,
        "MANIPULATION_VERB_DROP_OFF": 2,
        "MANIPULATION_PHASE_UNSPECIFIED": 0,
        "MANIPULATION_PHASE_APPROVED": 1,
        "MANIPULATION_PHASE_IN_PROGRESS": 2,
        "MANIPULATION_PHASE_SUCCEEDED": 3,
        "MANIPULATION_PHASE_FAILED": 4,
        "MANIPULATION_FAILURE_REASON_UNSPECIFIED": 0,
        "MANIPULATION_FAILURE_REASON_SAFETY_STOP": 1,
        "MANIPULATION_FAILURE_REASON_NO_OBJECT_DETECTED": 2,
        "MANIPULATION_FAILURE_REASON_UNSAFE_GRIP_HEIGHT": 3,
        "OBJECT_LIFECYCLE_UNSPECIFIED": 0,
        "OBJECT_LIFECYCLE_FREE": 1,
        "OBJECT_LIFECYCLE_HELD": 2,
        "MANIPULATION_GRASP_PROFILE_UNSPECIFIED": 0,
        "MANIPULATION_GRASP_PROFILE_TOP_DOWN": 1,
        "MANIPULATION_GRASP_PROFILE_MOCAP_X_AXIS": 2,
    }
    module.__dict__.update(constants)
    module.__dict__.update(
        Vec3=Vec3,
        Quat=QuatMessage,
        PoseXYZYaw=PoseXYZYaw,
        SE2VelocitySetpoint=SE2VelocitySetpoint,
        PlatformCommand=PlatformCommand,
        ManipulationCommand=ManipulationCommand,
        ManipulationStatus=ManipulationStatus,
        CommandStatus=CommandStatus,
        ConstraintEvent=ConstraintEvent,
        AgentStateMsg=AgentStateMsg,
        ObjectState=ObjectState,
        RigidBodyPose=RigidBodyPose,
        MocapRigidBodyFrame=MocapRigidBodyFrame,
        FrameCalibration=FrameCalibration,
    )
    return module


class FakeQuat:
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = float(w)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    @classmethod
    def from_yaw(cls, angle):
        return cls(w=math.cos(angle / 2.0), z=math.sin(angle / 2.0))

    @classmethod
    def from_pitch(cls, angle):
        return cls(w=math.cos(angle / 2.0), y=math.sin(angle / 2.0))

    @classmethod
    def from_matrix(cls, matrix):
        x, y, z, w = rot3_to_quat(matrix)
        return cls(w=w, x=x, y=y, z=z)

    def normalize(self):
        norm = math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        if norm == 0.0:
            raise ValueError("zero quaternion")
        self.w /= norm
        self.x /= norm
        self.y /= norm
        self.z /= norm
        return self

    def inverse(self):
        return FakeQuat(w=self.w, x=-self.x, y=-self.y, z=-self.z).normalize()

    def __mul__(self, other):
        return FakeQuat(
            w=self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x=self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y=self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z=self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        )

    def to_matrix(self):
        return quat_to_rot3(self.x, self.y, self.z, self.w)

    def to_yaw(self):
        q = FakeQuat(self.w, self.x, self.y, self.z).normalize()
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def transform_point(self, x, y, z):
        matrix = self.to_matrix()
        return tuple(sum(matrix[row][col] * (x, y, z)[col] for col in range(3)) for row in range(3))


class FakeTransform:
    def __init__(self, x=0.0, y=0.0, z=0.0, rot=None):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.rot = rot or FakeQuat()

    @classmethod
    def from_matrix(cls, matrix):
        flat = [value for row in matrix for value in row]
        return _mat4_to_se3pose(flat)

    def mult(self, other):
        rx, ry, rz = self.rot.transform_point(other.x, other.y, other.z)
        tx = rx + self.x
        ty = ry + self.y
        tz = rz + self.z
        new_rot = self.rot * other.rot
        return FakeTransform(tx, ty, tz, new_rot)

    def inverse(self):
        inverse_rotation = self.rot.inverse()
        ix, iy, iz = inverse_rotation.transform_point(-self.x, -self.y, -self.z)
        return FakeTransform(ix, iy, iz, inverse_rotation)

    def transform_point(self, x, y, z):
        rx, ry, rz = self.rot.transform_point(x, y, z)
        return rx + self.x, ry + self.y, rz + self.z

    def to_proto(self):
        return GeometrySE3Pose(
            position=GeometryVec3(x=self.x, y=self.y, z=self.z),
            rotation=GeometryQuaternion(x=self.rot.x, y=self.rot.y, z=self.rot.z, w=self.rot.w),
        )


def quat_to_rot3(qx, qy, qz, qw):
    q = FakeQuat(w=qw, x=qx, y=qy, z=qz).normalize()
    xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
    xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
    wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def rot3_to_quat(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale
    q = FakeQuat(w=w, x=x, y=y, z=z).normalize()
    return q.x, q.y, q.z, q.w


def se3_to_mat4(rotation, translation):
    return [
        rotation[0][0], rotation[0][1], rotation[0][2], translation[0],
        rotation[1][0], rotation[1][1], rotation[1][2], translation[1],
        rotation[2][0], rotation[2][1], rotation[2][2], translation[2],
        0.0, 0.0, 0.0, 1.0,
    ]


def mat4_compose(left, right):
    result = [0.0] * 16
    for row in range(4):
        for col in range(4):
            result[row * 4 + col] = sum(left[row * 4 + k] * right[k * 4 + col] for k in range(4))
    return result


def mat4_apply_point(matrix, point):
    x, y, z = point
    return tuple(
        matrix[row * 4] * x
        + matrix[row * 4 + 1] * y
        + matrix[row * 4 + 2] * z
        + matrix[row * 4 + 3]
        for row in range(3)
    )


def mat4_apply_vector(matrix, vector):
    x, y, z = vector
    return tuple(
        matrix[row * 4] * x + matrix[row * 4 + 1] * y + matrix[row * 4 + 2] * z
        for row in range(3)
    )


def se3_pose_to_mat4(pose):
    return se3_to_mat4(quat_to_rot3(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w), (pose.x, pose.y, pose.z))


def mat4_from_yaw_t(theta, translation):
    c, s = math.cos(theta), math.sin(theta)
    return se3_to_mat4([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], translation)


def mat4_from_yaw_t_inv(theta, translation):
    return mat4_compose(
        mat4_from_yaw_t(-theta, (0.0, 0.0, 0.0)),
        mat4_from_yaw_t(0.0, (-translation[0], -translation[1], -translation[2])),
    )


def _mat4_to_se3pose(matrix):
    tx, ty, tz = matrix[3], matrix[7], matrix[11]
    r = [
        [matrix[0], matrix[1], matrix[2]],
        [matrix[4], matrix[5], matrix[6]],
        [matrix[8], matrix[9], matrix[10]],
    ]
    qx, qy, qz, qw = rot3_to_quat(r)
    return FakeTransform(tx, ty, tz, FakeQuat(w=qw, x=qx, y=qy, z=qz))


def _se3pose_to_mat4(pose):
    return se3_pose_to_mat4(pose)


class Vec2(Struct):
    def __init__(self, x=0.0, y=0.0, **values):
        super().__init__(x=x, y=y, **values)


class GeometryVec3(Struct):
    def __init__(self, x=0.0, y=0.0, z=0.0, **values):
        super().__init__(x=x, y=y, z=z, **values)


class GeometryQuaternion(Struct):
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0, **values):
        super().__init__(w=w, x=x, y=y, z=z, **values)


class GeometrySE3Pose(Struct):
    def __init__(self, position=None, rotation=None, **values):
        super().__init__(position=position or GeometryVec3(), rotation=rotation or GeometryQuaternion(), **values)


class Value(Struct):
    def __init__(self, value=None):
        super().__init__(value=value)


def command_record(kind, **values):
    return Struct(kind=kind, **values)


class RobotCommandBuilder:
    @staticmethod
    def stop_command():
        return command_record("stop")

    @staticmethod
    def arm_ready_command():
        return command_record("ready")

    @staticmethod
    def arm_carry_command():
        return command_record("carry")

    @staticmethod
    def arm_stow_command():
        return command_record("stow")

    @staticmethod
    def claw_gripper_open_fraction_command(fraction):
        return command_record("open", fraction=fraction)

    @staticmethod
    def claw_gripper_close_command():
        return command_record("close")

    @staticmethod
    def arm_cartesian_move_helper(se3_poses, times, root_frame_name, wrist_tform_tool=None):
        cartesian = Struct(
            pose_trajectory_in_task=Struct(points=[Struct(pose=se3_poses[0])]),
            max_pos_tracking_error=Value(),
            root_frame_name=root_frame_name,
            times=list(times),
            wrist_tform_tool=wrist_tform_tool,
        )
        return command_record(
            "cartesian",
            synchronized_command=Struct(
                arm_command=Struct(arm_cartesian_command=cartesian)
            ),
        )

    @staticmethod
    def arm_pose_command_from_pose(hand_pose, frame_name, seconds=5, build_on_command=None):
        cartesian = Struct(
            pose_trajectory_in_task=Struct(points=[Struct(pose=hand_pose)]),
            root_frame_name=frame_name,
            seconds=seconds,
            wrist_tform_tool=GeometrySE3Pose(),
        )
        command = command_record(
            "cartesian",
            synchronized_command=Struct(
                arm_command=Struct(arm_cartesian_command=cartesian)
            ),
        )
        if build_on_command is not None:
            command.build_on_command = build_on_command
        return command

    @staticmethod
    def synchro_stand_command(params, build_on_command):
        build_on_command.mobility_params = params
        return build_on_command

    @staticmethod
    def synchro_velocity_command(
        v_x, v_y, v_rot, params=None, body_height=0.0,
        locomotion_hint=1, frame_name="body", build_on_command=None,
    ):
        # Real-builder output shape: the sidecar reads
        # command.synchronized_command.mobility_command.se2_velocity_request.
        # Default slew (4,4)/2.0 mirrors what the real builder hardcodes.
        velocity_request = Struct(
            se2_frame_name=frame_name,
            velocity=Struct(linear=Struct(x=v_x, y=v_y), angular=v_rot),
            slew_rate_limit=Struct(linear=Struct(x=4.0, y=4.0), angular=2.0),
        )
        command = command_record(
            "velocity",
            synchronized_command=Struct(
                mobility_command=Struct(se2_velocity_request=velocity_request)
            ),
        )
        command.mobility_params = params
        return command

    @staticmethod
    def mobility_params():
        # Mirror the real builder: locomotion_hint defaults to HINT_AUTO (1); vel_limit
        # is an empty message the caller fills via CopyFrom (Struct.CopyFrom stores it).
        return Struct(vel_limit=Struct(), locomotion_hint=1)


class ArmCartesianFeedback:
    STATUS_UNKNOWN = 0
    STATUS_TRAJECTORY_COMPLETE = 1
    STATUS_TRAJECTORY_STALLED = 2
    STATUS_TRAJECTORY_CANCELLED = 3
    STATUS_TRAJECTORY_IN_PROGRESS = 4


class NamedArmFeedback:
    STATUS_UNKNOWN = 0
    STATUS_COMPLETE = 1
    STATUS_STALLED_HOLDING_ITEM = 2
    STATUS_IN_PROGRESS = 3


class GripperFeedback:
    STATUS_UNKNOWN = 0
    STATUS_AT_GOAL = 1
    STATUS_APPLYING_FORCE = 2
    STATUS_IN_PROGRESS = 3


class TrajectoryFeedback:
    STATUS_UNKNOWN = 0
    STATUS_STOPPED = 1
    STATUS_GOING_TO_GOAL = 2
    FINAL_GOAL_STATUS_UNKNOWN = 0
    FINAL_GOAL_STATUS_ACHIEVABLE = 1
    FINAL_GOAL_STATUS_BLOCKED = 2

    class FinalGoalStatus:
        @staticmethod
        def Name(value):
            return {
                0: "FINAL_GOAL_STATUS_UNKNOWN",
                1: "FINAL_GOAL_STATUS_ACHIEVABLE",
                2: "FINAL_GOAL_STATUS_BLOCKED",
            }.get(value, f"UNRECOGNIZED_{value}")


def feedback_response(
    *,
    cartesian=ArmCartesianFeedback.STATUS_TRAJECTORY_COMPLETE,
    named=NamedArmFeedback.STATUS_COMPLETE,
    gripper=GripperFeedback.STATUS_AT_GOAL,
    trajectory=TrajectoryFeedback.STATUS_GOING_TO_GOAL,
    final_goal=TrajectoryFeedback.FINAL_GOAL_STATUS_UNKNOWN,
):
    return namespace(
        feedback=namespace(
            synchronized_feedback=namespace(
                arm_command_feedback=namespace(
                    arm_cartesian_feedback=namespace(status=cartesian),
                    named_arm_position_feedback=namespace(status=named),
                ),
                gripper_command_feedback=namespace(
                    claw_gripper_feedback=namespace(status=gripper)
                ),
                mobility_command_feedback=namespace(
                    se2_trajectory_feedback=namespace(
                        status=trajectory, final_goal_status=final_goal
                    )
                ),
            )
        )
    )


class FakeFuture:
    def __init__(self, error=None, done=True, value=None):
        self.error = error
        self.is_done = done
        self.value = value
        self.callbacks = []

    def done(self):
        return self.is_done

    def result(self):
        if self.error is not None:
            raise self.error
        return self.value

    def exception(self):
        return self.error

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.is_done:
            callback(self)


class FakeCommandClient:
    def __init__(self):
        self.commands = []
        self.command_attempts = []
        self.async_commands = []
        self.feedback_calls = []
        self.dispatch_results = deque()
        self.async_results = deque()
        self.feedback_results = deque()
        self.next_id = 1

    def robot_command(self, command, **kwargs):
        self.command_attempts.append((command, kwargs))
        result = self.dispatch_results.popleft() if self.dispatch_results else None
        if isinstance(result, BaseException):
            raise result
        command_id = result if result is not None else self.next_id
        self.next_id += 1
        self.commands.append((command, kwargs, command_id))
        return command_id

    def robot_command_async(self, command, **kwargs):
        self.command_attempts.append((command, kwargs))
        result = self.dispatch_results.popleft() if self.dispatch_results else None
        if isinstance(result, BaseException):
            return FakeFuture(error=result)
        return FakeFuture()

    def robot_command_feedback(self, command_id):
        self.feedback_calls.append(command_id)
        result = self.feedback_results.popleft() if self.feedback_results else feedback_response()
        if isinstance(result, BaseException):
            raise result
        return result


class FakeStateClient:
    def __init__(self, state=None):
        self.state = state or robot_state()
        self.calls = 0
        self.async_futures = []

    def get_robot_state(self):
        self.calls += 1
        return self.state

    def get_robot_state_async(self):
        self.calls += 1
        future = FakeFuture(value=self.state)
        self.async_futures.append(future)
        return future


class FakeManipulationApiClient:
    def __init__(self):
        self.commands = []
        self.feedback_calls = []
        self.feedback_responses = deque()
        self.next_id = 1

    def manipulation_api_command(self, manipulation_api_request, **kwargs):
        cmd_response = Struct(manipulation_cmd_id=self.next_id)
        self.commands.append((manipulation_api_request, kwargs, cmd_response.manipulation_cmd_id))
        self.next_id += 1
        return cmd_response

    def manipulation_api_feedback_command(self, manipulation_api_feedback_request, **kwargs):
        self.feedback_calls.append(manipulation_api_feedback_request.manipulation_cmd_id)
        if self.feedback_responses:
            return self.feedback_responses.popleft()
        return Struct(current_state=6)


class FakeRobot:
    def __init__(self, command_client=None, state_client=None, manipulation_client=None):
        self.command_client = command_client or FakeCommandClient()
        self.state_client = state_client or FakeStateClient()
        self.manipulation_client = manipulation_client or FakeManipulationApiClient()

    def ensure_client(self, service_name):
        if service_name == "robot-command":
            return self.command_client
        if service_name == "robot-state":
            return self.state_client
        if service_name == "manipulation":
            return self.manipulation_client
        raise AssertionError(f"unexpected service {service_name}")


class FakeEstopState(Struct):
    STATE_ESTOPPED = 1


class FakePowerState(Struct):
    STATE_OFF = 1
    STATE_ON = 2


class FakeBehaviorFault(Struct):
    pass


class FakeSystemFault(Struct):
    SEVERITY_INFO = 0
    SEVERITY_WARN = 1
    SEVERITY_CRITICAL = 2


def robot_state(
    *,
    transform=None,
    motors_on=True,
    estopped=False,
    behavior_fault_ids=(),
    system_faults=(),
):
    estops = [FakeEstopState(state=FakeEstopState.STATE_ESTOPPED)] if estopped else []
    behavior = [FakeBehaviorFault(behavior_fault_id=value) for value in behavior_fault_ids]
    systems = [FakeSystemFault(name=name, severity=severity) for name, severity in system_faults]
    return namespace(
        power_state=FakePowerState(
            motor_power_state=(
                FakePowerState.STATE_ON if motors_on else FakePowerState.STATE_OFF
            )
        ),
        estop_states=estops,
        behavior_fault_state=namespace(faults=behavior),
        system_fault_state=namespace(faults=systems),
        kinematic_state=namespace(
            transforms_snapshot=transform or FakeTransform(),
            velocity_of_body_in_vision=namespace(
                linear=namespace(x=0.0, y=0.0, z=0.0),
                angular=namespace(x=0.0, y=0.0, z=0.0),
            ),
        ),
    )


class CapturePublisher:
    def __init__(self):
        self.payloads = []

    def put(self, payload):
        self.payloads.append(bytes(payload))

    def decoded(self, message_type):
        return [message_type.FromString(payload) for payload in self.payloads]


class Sample:
    def __init__(self, message):
        self.payload = message.SerializeToString()


class ManualClock:
    def __init__(self, start=100.0):
        self.value = float(start)

    def time(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


def _module(name, **attributes):
    module = types.ModuleType(name)
    module.__dict__.update(attributes)
    return module


class FakeArray(list):
    def reshape(self, rows, columns):
        flat = (
            [value for row in self for value in row]
            if self and isinstance(self[0], list)
            else self
        )
        if len(flat) != rows * columns:
            raise ValueError("array cannot be reshaped to the requested dimensions")
        return [
            list(flat[row * columns : (row + 1) * columns])
            for row in range(rows)
        ]


def install_fake_modules():
    pb = build_fake_protobuf_module()
    sys.modules["core_msgs_pb2"] = pb



    zenoh = _module(
        "zenoh",
        Config=FakeZenohConfig,
        CongestionControl=namespace(DROP=1, BLOCK=2),
        Priority=namespace(REAL_TIME=1),
        open=lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("tests must not open Zenoh")
        ),
    )
    sys.modules["zenoh"] = zenoh

    numpy = _module("numpy", array=lambda values, **_kwargs: FakeArray(values))
    sys.modules["numpy"] = numpy

    any_pb2 = _module("google.protobuf.any_pb2")

    class AnyValue(Struct):
        def __init__(self):
            super().__init__(packed=None)

        def Pack(self, value):
            self.packed = value

    any_pb2.Any = AnyValue
    class Descriptor(Struct): pass
    class Message(Struct): pass
    descriptor = _module("google.protobuf.descriptor", Descriptor=Descriptor, FieldDescriptor=Struct, FileDescriptor=Struct, EnumDescriptor=Struct, EnumValueDescriptor=Struct, DescriptorProto=Struct, FileDescriptorProto=Struct, MessageOptions=Struct, FieldOptions=Struct, EnumOptions=Struct, EnumValueOptions=Struct, ServiceOptions=Struct, MethodOptions=Struct, OneofDescriptor=Struct, OneofOptions=Struct)
    descriptor_pool = _module("google.protobuf.descriptor_pool", Default=lambda: Struct(AddSerializedFile=lambda x: None))
    runtime_version = _module("google.protobuf.runtime_version", Domain=Struct(PUBLIC=1), ValidateProtobufRuntimeVersion=lambda *args: None)
    message = _module("google.protobuf.message", Message=Message)
    reflection = _module("google.protobuf.reflection", GeneratedProtocolMessageType=type)
    symbol_database = _module("google.protobuf.symbol_database", Default=lambda: Struct(RegisterMessage=lambda x: x, RegisterEnumDescriptor=lambda x: x))
    internal = _module("google.protobuf.internal")
    builder = _module("google.protobuf.internal.builder", BuildMessageAndEnumDescriptors=lambda *args: None, BuildTopDescriptorsAndMessages=lambda *args: None)
    internal.builder = builder

    google = _module("google")
    google.__path__ = []
    protobuf = _module("google.protobuf", any_pb2=any_pb2, descriptor=descriptor, descriptor_pool=descriptor_pool, runtime_version=runtime_version, message=message, reflection=reflection, symbol_database=symbol_database, internal=internal)
    protobuf.__path__ = []
    google.protobuf = protobuf
    sys.modules.update(
        {
            "google": google,
            "google.protobuf": protobuf,
            "google.protobuf.any_pb2": any_pb2,
            "google.protobuf.descriptor": descriptor,
            "google.protobuf.descriptor_pool": descriptor_pool,
            "google.protobuf.runtime_version": runtime_version,
            "google.protobuf.message": message,
            "google.protobuf.reflection": reflection,
            "google.protobuf.symbol_database": symbol_database,
            "google.protobuf.internal": internal,
            "google.protobuf.internal.builder": builder,
        }
    )

    geometry_pb2 = _module(
        "bosdyn.api.geometry_pb2",
        Vec2=Vec2,
        Vec3=GeometryVec3,
        Quaternion=GeometryQuaternion,
        SE2Pose=Struct,
        SE3Pose=GeometrySE3Pose,
        SE2Velocity=Struct,
        SE2VelocityLimit=Struct,
    )

    class ArmCartesianCommand:
        Feedback = ArmCartesianFeedback

    class NamedArmPositionsCommand:
        Feedback = NamedArmFeedback

    class ClawGripperCommand:
        Feedback = GripperFeedback

    arm_command_pb2 = _module(
        "bosdyn.api.arm_command_pb2",
        ArmCartesianCommand=ArmCartesianCommand,
        NamedArmPositionsCommand=NamedArmPositionsCommand,
    )
    gripper_command_pb2 = _module(
        "bosdyn.api.gripper_command_pb2", ClawGripperCommand=ClawGripperCommand
    )

    class BodyAssistForManipulation(Struct):
        pass

    class BodyControlParams(Struct):
        pass

    BodyControlParams.BodyAssistForManipulation = BodyAssistForManipulation

    class MobilityParams(Struct):
        pass

    spot_robot_command_pb2 = _module(
        "bosdyn.api.spot.robot_command_pb2",
        MobilityParams=MobilityParams,
        BodyControlParams=BodyControlParams,
        HINT_AUTO=1,
        HINT_AMBLE=5,
    )

    class SE2TrajectoryCommand:
        Feedback = TrajectoryFeedback
        Request = Struct

    class SE2VelocityCommand:
        Request = Struct

    basic_command_pb2 = _module(
        "bosdyn.api.basic_command_pb2",
        SE2TrajectoryCommand=SE2TrajectoryCommand,
        SE2VelocityCommand=SE2VelocityCommand,
    )

    class MobilityCommand:
        Request = Struct

    mobility_command_pb2 = _module(
        "bosdyn.api.mobility_command_pb2", MobilityCommand=MobilityCommand
    )

    class SynchronizedCommand:
        Request = Struct

    synchronized_command_pb2 = _module(
        "bosdyn.api.synchronized_command_pb2", SynchronizedCommand=SynchronizedCommand
    )
    trajectory_pb2 = _module(
        "bosdyn.api.trajectory_pb2",
        SE2Trajectory=Struct,
        SE2TrajectoryPoint=Struct,
    )
    robot_command_pb2 = _module("bosdyn.api.robot_command_pb2", RobotCommand=Struct)

    manipulation_api_pb2 = _module(
        "bosdyn.api.manipulation_api_pb2",
        GraspParams=Struct,
        PickObject=Struct,
        ManipulationApiRequest=Struct,
        ManipulationApiFeedbackRequest=Struct,
        MANIP_STATE_UNKNOWN=0,
        MANIP_STATE_DONE=1,
        MANIP_STATE_SEARCHING_FOR_GRASP=2,
        MANIP_STATE_MOVING_TO_GRASP=3,
        MANIP_STATE_GRASPING_OBJECT=4,
        MANIP_STATE_PLACING_OBJECT=5,
        MANIP_STATE_GRASP_SUCCEEDED=6,
        MANIP_STATE_GRASP_FAILED=7,
        MANIP_STATE_GRASP_PLANNING_SUCCEEDED=11,
        MANIP_STATE_GRASP_PLANNING_NO_SOLUTION=8,
        MANIP_STATE_GRASP_FAILED_TO_RAYCAST_INTO_MAP=9,
        MANIP_STATE_GRASP_PLANNING_WAITING_DATA_AT_EDGE=13,
        MANIP_STATE_WALKING_TO_OBJECT=10,
        MANIP_STATE_ATTEMPTING_RAYCASTING=12,
        MANIP_STATE_MOVING_TO_PLACE=14,
        MANIP_STATE_PLACE_FAILED_TO_RAYCAST_INTO_MAP=15,
        MANIP_STATE_PLACE_SUCCEEDED=16,
        MANIP_STATE_PLACE_FAILED=17,
    )
    class ManipulationFeedbackState:
        @staticmethod
        def Name(value):
            return {
                0: "MANIP_STATE_UNKNOWN",
                1: "MANIP_STATE_DONE",
                2: "MANIP_STATE_SEARCHING_FOR_GRASP",
                3: "MANIP_STATE_MOVING_TO_GRASP",
                4: "MANIP_STATE_GRASPING_OBJECT",
                5: "MANIP_STATE_PLACING_OBJECT",
                6: "MANIP_STATE_GRASP_SUCCEEDED",
                7: "MANIP_STATE_GRASP_FAILED",
                8: "MANIP_STATE_GRASP_PLANNING_NO_SOLUTION",
                9: "MANIP_STATE_GRASP_FAILED_TO_RAYCAST_INTO_MAP",
                10: "MANIP_STATE_WALKING_TO_OBJECT",
                11: "MANIP_STATE_GRASP_PLANNING_SUCCEEDED",
                12: "MANIP_STATE_ATTEMPTING_RAYCASTING",
                13: "MANIP_STATE_GRASP_PLANNING_WAITING_DATA_AT_EDGE",
                14: "MANIP_STATE_MOVING_TO_PLACE",
                15: "MANIP_STATE_PLACE_FAILED_TO_RAYCAST_INTO_MAP",
                16: "MANIP_STATE_PLACE_SUCCEEDED",
                17: "MANIP_STATE_PLACE_FAILED",
            }.get(value, f"UNRECOGNIZED_{value}")
    manipulation_api_pb2.ManipulationFeedbackState = ManipulationFeedbackState

    api = _module(
        "bosdyn.api",
        arm_command_pb2=arm_command_pb2,
        basic_command_pb2=basic_command_pb2,
        geometry_pb2=geometry_pb2,
        gripper_command_pb2=gripper_command_pb2,
        mobility_command_pb2=mobility_command_pb2,
        robot_command_pb2=robot_command_pb2,
        synchronized_command_pb2=synchronized_command_pb2,
        trajectory_pb2=trajectory_pb2,
        manipulation_api_pb2=manipulation_api_pb2,
    )
    api.__path__ = []
    spot_api = _module("bosdyn.api.spot", robot_command_pb2=spot_robot_command_pb2)
    spot_api.__path__ = []

    frame_helpers = _module(
        "bosdyn.client.frame_helpers",
        BODY_FRAME_NAME="body",
        FOOT_FRAME_NAMES=("fl", "fr", "hl", "hr"),
        GRAV_ALIGNED_BODY_FRAME_NAME="flat_body",
        GROUND_PLANE_FRAME_NAME="gpe",
        VISION_FRAME_NAME="vision",
        get_a_tform_b=lambda snapshot, _parent, _child: snapshot,
    )

    class RobotCommandClient:
        default_service_name = "robot-command"

    class RobotStateClient:
        default_service_name = "robot-state"

    robot_command = _module(
        "bosdyn.client.robot_command",
        RobotCommandBuilder=RobotCommandBuilder,
        RobotCommandClient=RobotCommandClient,
        blocking_stand=lambda *_args, **_kwargs: None,
    )
    robot_state_module = _module(
        "bosdyn.client.robot_state", RobotStateClient=RobotStateClient
    )

    class LeaseClient:
        default_service_name = "lease"

    class ResourceAlreadyClaimedError(Exception):
        def __init__(self, response=None):
            super().__init__("body lease is already claimed")
            self.response = response

    class LeaseKeepAlive:
        def __init__(self, *_args, **_kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

    lease = _module(
        "bosdyn.client.lease",
        LeaseClient=LeaseClient,
        LeaseKeepAlive=LeaseKeepAlive,
        ResourceAlreadyClaimedError=ResourceAlreadyClaimedError,
    )
    class KeepaliveMotorsOffError(Exception):
        pass

    power = _module(
        "bosdyn.client.power",
        KeepaliveMotorsOffError=KeepaliveMotorsOffError,
    )
    math_helpers = _module(
        "bosdyn.client.math_helpers", Quat=FakeQuat, SE3Pose=FakeTransform
    )
    util = _module("bosdyn.client.util", authenticate=lambda _robot: None)

    class ManipulationApiClient:
        default_service_name = "manipulation"

    manipulation_api_client_module = _module(
        "bosdyn.client.manipulation_api_client", ManipulationApiClient=ManipulationApiClient
    )

    client = _module(
        "bosdyn.client",
        frame_helpers=frame_helpers,
        robot_command=robot_command,
        robot_state=robot_state_module,
        lease=lease,
        power=power,
        math_helpers=math_helpers,
        util=util,
        manipulation_api_client=manipulation_api_client_module,
        create_standard_sdk=lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("tests must not create an SDK session")
        ),
    )
    client.__path__ = []
    bosdyn = _module("bosdyn", client=client, api=api)
    bosdyn.__path__ = []

    sys.modules.update(
        {
            "bosdyn": bosdyn,
            "bosdyn.api": api,
            "bosdyn.api.arm_command_pb2": arm_command_pb2,
            "bosdyn.api.basic_command_pb2": basic_command_pb2,
            "bosdyn.api.geometry_pb2": geometry_pb2,
            "bosdyn.api.gripper_command_pb2": gripper_command_pb2,
            "bosdyn.api.mobility_command_pb2": mobility_command_pb2,
            "bosdyn.api.robot_command_pb2": robot_command_pb2,
            "bosdyn.api.synchronized_command_pb2": synchronized_command_pb2,
            "bosdyn.api.trajectory_pb2": trajectory_pb2,
            "bosdyn.api.manipulation_api_pb2": manipulation_api_pb2,
            "bosdyn.api.spot": spot_api,
            "bosdyn.api.spot.robot_command_pb2": spot_robot_command_pb2,
            "bosdyn.client": client,
            "bosdyn.client.frame_helpers": frame_helpers,
            "bosdyn.client.lease": lease,
            "bosdyn.client.math_helpers": math_helpers,
            "bosdyn.client.power": power,
            "bosdyn.client.robot_command": robot_command,
            "bosdyn.client.robot_state": robot_state_module,
            "bosdyn.client.util": util,
            "bosdyn.client.manipulation_api_client": manipulation_api_client_module,
        }
    )
    return pb


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_production_modules():
    """Loads fresh arm and sidecar modules against the hermetic fakes."""

    pb = install_fake_modules()
    sys.modules.pop("spot_sidecar_under_test", None)
    spot_sidecar = _load_module(
        "spot_sidecar_under_test", SIDECAR_ROOT / "spot_sidecar.py"
    )
    return namespace(pb=pb, spot_sidecar=spot_sidecar)
