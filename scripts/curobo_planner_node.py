#!/usr/bin/env python3
"""cuRobo Motion Planner Node for Doosan E0509

Pick sequence: pre-approach(CuRobo) → straight grasp(MoveLine) → close
               → straight reverse retreat(MoveLine) → pick-start scan pose → pick_complete
"""

import os
import time
import torch
import numpy as np
import json
import yaml
import glob
from scipy.spatial.transform import Rotation as SciR

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, String, Empty, Int32
from std_srvs.srv import Trigger
from dsr_msgs2.srv import MoveSplineJoint, MoveJoint, MoveLine

from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState as CuroboJointState, RobotConfig
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.geom.types import WorldConfig, Cuboid, Sphere
from runtime_jsonl_logger import RuntimeJsonlLogger


# ── 파지 파라미터 ──────────────────────────────────────────────────────────────
GRASP_RETRY_OFFSETS  = [0.015, 0.030, 0.040, 0.050, 0.070]
MEASURED_TCP_FINAL_STANDOFF_M = -0.120  # 실기 관찰: 30mm에서 15cm 부족 → 180mm 접근으로 조정 (추후 실측 후 수정)
Y_DETECTION_BIAS_M = 0.000  # 보정값 0: raw detection Y를 그대로 접근 거리 계산에 사용
                              # (단일 데이터 포인트 기반 23mm 추정은 신뢰 부족 → 실측 후 재조정)
LEFTMOST_GRASP_RETRY_OFFSETS = [0.030, 0.035, 0.040, 0.045, 0.050, 0.070]
LEFTMOST_GRASP_X_CORR_M = 0.005   # x < -300mm: +X 보정 (ELBOW_UP 드리프트 보정)
LEFTMOST_EXTRA_ADVANCE_REQUEST_M = 0.065  # 실기 요청값; wall safety gate가 실제 실행량을 제한
LEFTMOST_WALL_SAFETY_MARGIN_M = -0.030   # 실기 확인: 줄기가 모델 벽 30mm 안쪽 → 음수로 80mm extra 허용
# 근거: 2026-06-09 x=-345mm target, 210mm 진입 성공, 역진 정상
# available_extra = grasp_offset(50mm) - margin(-30mm) = 80mm → override 불필요
LEFTMOST_EXTRA_ADVANCE_VEL_MM_S = 50.0    # 직접 접근이므로 고속 진입
GRASP_Z_BIAS             = 0.020    # +20mm: 잎 위에서 줄기 상단부 파지, BASE -Z 당기기로 분리
PRE_APPROACH_OFFSET      = 0.06     # 6cm 접근 재검증: 직전 측방 편차가 줄기 형상 영향인지 분리
PRE_APPROACH_SETTLE_SEC  = 0.5      # spline 잔진동이 멈춘 뒤 직선 접근 시작
FINAL_APPROACH_VEL_MM_S  = 50.0
FINAL_APPROACH_ACC_MM_S2 = 60.0
RETREAT_VEL_MM_S         = 80.0  # 직선 retreat — approach보다 고속으로 줄기 분리
RETREAT_ACC_MM_S2         = 100.0
STRAIGHT_RETREAT_SETTLE_SEC = 0.5
NEIGHBOR_SPHERE_RADIUS_M = 0.030
# ── 크레인 접근 (위에서 아래로 진입) ─────────────────────────────────────────
# pre-approach를 목표 z보다 높은 위치에서 잡고, BASE -Z로 내려온 뒤 TOOL +Z 진입.
# 잎 canopy 위에서 접근해 높이/위치와 무관하게 일관된 잎 회피.
CRANE_Z_OFFSET_M      = 0.000   # 크레인 접근 비활성화 (0이면 관련 코드 전체 skip)
CRANE_DESCENT_VEL_MM_S = 30.0
CRANE_ASCENT_VEL_MM_S  = 50.0
DETACH_PULL_DOWN_MM  = 40.0   # 파지 후 BASE -Z 당기기 거리 (mm)
DETACH_PULL_VEL_MM_S = 50.0   # 저속 분리 (줄기 충격 방지)

# Tool geometry measured on 2026-06-11.
# Physical grasp center is about 10mm behind the part tips:
#   flange -> original gripper: 160mm
#   flange -> part tips:        270mm
#   flange -> grasp center:     260mm
#
# The current cuRobo URDF still places gripper_rh_p12_rn_base at link_6 and the
# proven SW baseline was tuned with a 160mm software offset plus extra advance.
# Keep that legacy offset until an explicit grasp_tcp_link and collision model
# are validated; changing it directly would shift physical motion by about 100mm.
LEGACY_EE_TO_TCP_OFFSET_M = 0.160
MEASURED_FLANGE_TO_GRIPPER_M = 0.160
MEASURED_FLANGE_TO_PART_TIP_M = 0.270
MEASURED_FLANGE_TO_GRASP_CENTER_M = 0.260
TCP_MODEL_SHORTFALL_M = (
    MEASURED_FLANGE_TO_GRASP_CENTER_M - LEGACY_EE_TO_TCP_OFFSET_M
)
WALL_SURFACE_Y_M = 0.672       # whiteboard 전면 Y — berry Y 클램핑 상한 (FK drift 보정)
WALL_QUAT_WXYZ   = [0.497, -0.497, 0.503, 0.503]   # approach_dir = [0, 1, 0] 정확히 수직
# 유도: [0.488, -0.506, 0.494, 0.512] (elevation 0°) 에 world-Z -2.06° 추가 보정
# → approach_dir X 성분(-0.036) 제거, 130mm 직선 접근 시 횡방향 오차 0mm
# 롤백: [0.548415, -0.439294, 0.424628, 0.570923] (원본 측정값)
GRASP_QUAT_RETRY_VARIANTS: list = [
    ("base",  [1, 0, 0], -10.0),  # 10° 아래 — 잎 위에서 진입, 잎 회피 1차 시도
    ("base",  [1, 0, 0],  -5.0),  # 5° 아래 (2차)
    ("base",  [1, 0, 0],   0.0),  # 수평 (3차 — 잎 많은 경우 밀릴 수 있음)
    ("base",  [1, 0, 0],  +5.0),  # 5° 위 (4차)
]
MEASURED_TCP_GRASP_QUAT_RETRY_VARIANTS: list = [
    ("base", [1, 0, 0], -10.0),
    ("base", [1, 0, 0],  -5.0),
    ("base", [1, 0, 0],   0.0),
    ("base", [1, 0, 0],  +5.0),
    ("base", [1, 0, 0], +10.0),
    ("base", [1, 0, 0], +15.0),
]

CARTESIAN_PLAN_MAX_ATTEMPTS = 1
CARTESIAN_PLAN_TIMEOUT_SEC  = 0.8
DIRECT_GRASP_TARGET_X_RANGE_M = (-0.45, 0.45)

GRIPPER_APPROACH_POS        = 600  # 접근 시 개도 (스캔 이동 중 미리 설정됨)

# ── 파지 검증 ─────────────────────────────────────────────────────────────────
# RH-P12-RN-A: 0=fully open, 700=fully closed
# 줄기가 잡히면 jaw가 중간에 멈춤(예: ~600~650). 아무것도 없으면 700까지 닫힘.
# 임계값 이상이면 GRASP_EMPTY 판정.
GRASP_EMPTY_POSITION_THRESHOLD = 665   # pos >= 665 → fully closed → nothing grabbed
GRASP_VERIFY_TIMEOUT_SEC       = 5.0   # read_position 서비스 타임아웃 (hardware read ~1.5s 포함)

# ── 고정 자세 ──────────────────────────────────────────────────────────────────
HOME_JOINTS_DEG     = [88.0,  -80.0, 130.0,   0.0, 20.0,  -90.0]
OVERVIEW_JOINTS_DEG = [87.98, -94.92, 129.89, 175.94, -31.34, 93.42]  # 스캔 기준 포즈
TRAY_VIEW_JOINTS_DEG = [-0.02, -2.41, 111.87, 175.94, -31.34, 93.42]
DEFAULT_TRAY_CELLS_GLOB = os.path.expanduser(
    "~/Downloads/share_tray/output/tray_cells_*.json")

# ── cuRobo 운용 한계 ───────────────────────────────────────────────────────────
OPERATIONAL_JOINT_LIMITS_DEG = [
    (-225.0, 225.0),   # J1
    (-95.0,   95.0),
    (-155.0, 155.0),
    (-280.0, 280.0),   # J4: SW scan pose=262.2°, retreat 시 274° 도달 → ±270° 시 normalize 불연속 발생
    (-130.0, 130.0),
    (-225.0, 225.0),
]
WRAP_EQUIVALENT_JOINT_IDX = {3, 5}   # J1은 정규화 금지 (반대 branch로 스윙)
MAX_HARVEST_JOINT_DELTA_DEG = [75.0, 90.0, 120.0, 150.0, 130.0, 180.0]

# ── Spline 실행 ────────────────────────────────────────────────────────────────
MAX_SPLINE_POINTS = 12
SPLINE_TIME_SCALE = 1.125   # cuRobo plan_time 배율 (1/3 속도)
SPLINE_MIN_TIME   = 0.75

USE_CUROBO_SELF_COLLISION = False   # coarse sphere 모델이 정상 자세도 오검출
DEBUG_START_COLLISION     = True    # INVALID_START_STATE_WORLD_COLLISION 원인 로그


def resolve_environment_yaml():
    candidates = [
        os.path.expanduser("~/doosan_ws/src/e0509_gripper_description/config/environment.yaml"),
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "environment.yaml",
        ),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]


ENVIRONMENT_YAML = resolve_environment_yaml()


def quat_multiply_wxyz(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ]


def quat_from_axis_angle(axis, angle_rad):
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = np.sin(angle_rad / 2.0)
    return [np.cos(angle_rad / 2.0), axis[0] * s, axis[1] * s, axis[2] * s]


def quat_rotate_vec(q_wxyz, v):
    """쿼터니언 q_wxyz=[w,x,y,z]으로 벡터 v를 회전."""
    w, x, y, z = q_wxyz
    qvec = np.array([x, y, z])
    v = np.array(v, dtype=float)
    t = 2.0 * np.cross(qvec, v)
    return v + w * t + np.cross(qvec, t)


def load_environment_cuboids():
    if not os.path.exists(ENVIRONMENT_YAML):
        return [Cuboid(name="table", pose=[0.0, 0.0, -0.02, 1, 0, 0, 0], dims=[1.2, 1.2, 0.04])]
    with open(ENVIRONMENT_YAML, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    cuboids = []
    for obj in data.get("objects", []):
        if not obj.get("enabled", True):
            continue
        if obj.get("type", "cuboid") != "cuboid":
            continue
        try:
            cuboids.append(Cuboid(
                name=str(obj["name"]),
                pose=[float(v) for v in obj["pose"]],
                dims=[float(v) for v in obj["dims"]],
            ))
        except Exception as e:
            print(f"[WARN] environment object skipped: {obj.get('name', '?')} ({e})")
    if not cuboids:
        cuboids.append(Cuboid(name="table", pose=[0.0, 0.0, -0.02, 1, 0, 0, 0], dims=[1.2, 1.2, 0.04]))
    return cuboids


class CuroboPlanner(Node):

    JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    JOINT_LIMITS = [
        (-6.273185, 6.273185),
        (-1.648063, 1.648063),
        (-2.6953,   2.6953  ),
        (-6.273185, 6.273185),
        (-2.346194, 2.346194),
        (-6.273185, 6.273185),
    ]

    def __init__(self):
        super().__init__("curobo_planner_node")

        self.runtime_log = RuntimeJsonlLogger(self.get_name())
        self.service_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.current_joints = None
        self._pick_busy = False
        self._sequence_hold_reason = None
        self._last_sequence_hold_warn_sec = 0.0
        self._marker_place_slot_idx = 0
        self.static_cuboids = load_environment_cuboids()
        self.dynamic_cuboids = []
        self.neighbor_spheres: list = []
        self._registered_neighbor_positions: list[np.ndarray] = []
        self._scene_positions: list = []

        # ── cuRobo 초기화 ──────────────────────────────────────────────────────
        self.declare_parameter("tool_model_profile", "measured_tcp_260mm")
        self._tool_model_profile = str(
            self.get_parameter("tool_model_profile").value).strip()
        if self._tool_model_profile not in {"measured_tcp_260mm", "legacy_160mm"}:
            raise ValueError(
                "tool_model_profile must be measured_tcp_260mm or legacy_160mm")
        self._measured_tcp_model = self._tool_model_profile == "measured_tcp_260mm"
        self._ee_to_tcp_offset_m = (
            0.0 if self._measured_tcp_model else LEGACY_EE_TO_TCP_OFFSET_M
        )
        self.declare_parameter("measured_tcp_plan_only", True)
        self._measured_tcp_plan_only = bool(
            self.get_parameter("measured_tcp_plan_only").value)
        config_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "curobo"
        )
        if not os.path.exists(config_dir):
            from ament_index_python.packages import get_package_share_directory
            config_dir = os.path.join(
                get_package_share_directory("e0509_gripper_description"),
                "config", "curobo"
            )

        tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
        robot_config_name = (
            "e0509_gripper_measured_tcp.yml"
            if self._measured_tcp_model
            else "e0509_gripper.yml"
        )
        with open(os.path.join(config_dir, robot_config_name), "r", encoding="utf-8") as f:
            robot_cfg_data = yaml.safe_load(f)
        robot_kin = robot_cfg_data["robot_cfg"]["kinematics"]
        robot_kin["urdf_path"] = os.path.join(config_dir, "e0509_gripper.urdf")
        robot_kin["collision_spheres"] = os.path.join(config_dir, "e0509_spheres.yml")
        robot_cfg = RobotConfig.from_dict(robot_cfg_data, tensor_args=tensor_args)
        world_cfg = WorldConfig(cuboid=self.static_cuboids)
        motion_gen_cfg = MotionGenConfig.load_from_robot_config(
            robot_cfg, world_cfg, tensor_args=tensor_args,
            num_trajopt_seeds=16, num_graph_seeds=16,
            collision_cache={"obb": 30, "mesh": 10, "sphere": 30},
            use_cuda_graph=False,
            self_collision_check=USE_CUROBO_SELF_COLLISION,
            self_collision_opt=USE_CUROBO_SELF_COLLISION,
        )
        self.motion_gen = MotionGen(motion_gen_cfg)
        self.motion_gen.warmup(warmup_js_trajopt=False)
        self.motion_gen.detach_object_from_robot()
        self.get_logger().info("cuRobo MotionGen warmed up!")

        self.declare_parameter("enable_marker_place_sequence", False)
        self.declare_parameter("execute_marker_place_release", False)
        self.declare_parameter("allow_unverified_grasp_place", False)
        self.declare_parameter("grasp_current_contact_threshold_raw", -1)
        self.declare_parameter("tray_cells_json", "")
        self.declare_parameter("marker_place_max_age_sec", 3600.0)
        self.declare_parameter("marker_place_above_clearance_m", 0.100)
        self.declare_parameter(
            "leftmost_extra_advance_request_m",
            0.0 if self._measured_tcp_model else LEFTMOST_EXTRA_ADVANCE_REQUEST_M)
        self.declare_parameter(
            "leftmost_wall_safety_margin_m", LEFTMOST_WALL_SAFETY_MARGIN_M)
        self.declare_parameter("leftmost_allow_wall_model_override", False)
        self._enable_marker_place = bool(
            self.get_parameter("enable_marker_place_sequence").value)
        self._execute_marker_place_release = bool(
            self.get_parameter("execute_marker_place_release").value)
        self._allow_unverified_grasp_place = bool(
            self.get_parameter("allow_unverified_grasp_place").value)
        self._grasp_current_contact_threshold_raw = int(
            self.get_parameter("grasp_current_contact_threshold_raw").value)
        self._tray_cells_json = os.path.expanduser(
            str(self.get_parameter("tray_cells_json").value))
        self._marker_place_max_age_sec = float(
            self.get_parameter("marker_place_max_age_sec").value)
        self._marker_place_above_clearance_m = float(
            self.get_parameter("marker_place_above_clearance_m").value)
        self._leftmost_extra_advance_request_m = max(
            0.0, float(self.get_parameter("leftmost_extra_advance_request_m").value))
        self._leftmost_wall_safety_margin_m = float(
            self.get_parameter("leftmost_wall_safety_margin_m").value)
        self._leftmost_allow_wall_model_override = bool(
            self.get_parameter("leftmost_allow_wall_model_override").value)

        # ── ROS2 인터페이스 ────────────────────────────────────────────────────
        self.create_subscription(
            JointState, "/dsr01/joint_states", self.joint_state_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            PoseStamped, "/dsr01/curobo/target_pose", self.target_pose_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            PoseStamped, "/dsr01/curobo/pick_pose", self.pick_pose_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            String, "/dsr01/curobo/obstacles", self.obstacles_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            Float64MultiArray, "/strawberry/detection/scene_positions", self._scene_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            String, "/strawberry/scan/status", self._scan_status_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            String, "/strawberry/exploration/set_cell_state", self._cell_state_cb, 10,
            callback_group=self.service_cb_group)

        self.pick_complete_pub = self.create_publisher(Empty, "/dsr01/curobo/pick_complete", 10)
        self.gripper_pos_pub = self.create_publisher(Int32, "/dsr01/gripper/position_cmd", 10)

        self.cli_spline = self.create_client(
            MoveSplineJoint, "/dsr01/motion/move_spline_joint",
            callback_group=self.service_cb_group)
        self.cli_movej = self.create_client(
            MoveJoint, "/dsr01/motion/move_joint",
            callback_group=self.service_cb_group)
        self.cli_movel = self.create_client(
            MoveLine, "/dsr01/motion/move_line",
            callback_group=self.service_cb_group)
        self.cli_gripper_open = self.create_client(
            Trigger, "/dsr01/gripper/open", callback_group=self.service_cb_group)
        self.cli_gripper_close = self.create_client(
            Trigger, "/dsr01/gripper/close", callback_group=self.service_cb_group)
        self.cli_gripper_read_state = self.create_client(
            Trigger, "/dsr01/gripper/read_state", callback_group=self.service_cb_group)

        self.get_logger().info("cuRobo Planner Ready!")
        self.get_logger().info(f"Runtime JSONL: {self.runtime_log.path}")
        self.runtime_log.log(
            "node_start",
            wall_quat_wxyz=WALL_QUAT_WXYZ,
            grasp_retry_offsets_m=GRASP_RETRY_OFFSETS,
            leftmost_grasp_retry_offsets_m=LEFTMOST_GRASP_RETRY_OFFSETS,
            leftmost_grasp_x_correction_m=LEFTMOST_GRASP_X_CORR_M,
            leftmost_extra_advance_request_m=self._leftmost_extra_advance_request_m,
            leftmost_wall_safety_margin_m=self._leftmost_wall_safety_margin_m,
            leftmost_allow_wall_model_override=self._leftmost_allow_wall_model_override,
            pre_approach_offset_m=PRE_APPROACH_OFFSET,
            tool_model_profile=self._tool_model_profile,
            measured_tcp_plan_only=self._measured_tcp_plan_only,
            ee_to_tcp_offset_m=self._ee_to_tcp_offset_m,
            legacy_ee_to_tcp_offset_m=LEGACY_EE_TO_TCP_OFFSET_M,
            measured_flange_to_gripper_m=MEASURED_FLANGE_TO_GRIPPER_M,
            measured_flange_to_part_tip_m=MEASURED_FLANGE_TO_PART_TIP_M,
            measured_flange_to_grasp_center_m=MEASURED_FLANGE_TO_GRASP_CENTER_M,
            tcp_model_shortfall_m=TCP_MODEL_SHORTFALL_M,
            enable_marker_place=self._enable_marker_place,
            execute_marker_place_release=self._execute_marker_place_release,
            allow_unverified_grasp_place=self._allow_unverified_grasp_place,
            grasp_current_contact_threshold_raw=self._grasp_current_contact_threshold_raw,
            marker_place_max_age_sec=self._marker_place_max_age_sec,
        )
        base_approach_dir = np.array(quat_rotate_vec(WALL_QUAT_WXYZ, [0.0, 0.0, 1.0]))
        base_elevation_deg = float(np.degrees(np.arcsin(np.clip(base_approach_dir[2], -1.0, 1.0))))
        self.get_logger().info(
            f"  ENV_CUBOIDS={len(self.static_cuboids)}  "
            f"SELF_COLLISION={USE_CUROBO_SELF_COLLISION}")
        self.get_logger().warn(
            "Leaf/stem geometry is not in the cuRobo world; visually occluded "
            "targets require reobserve/skip instead of forced approach")
        self.get_logger().info(
            f"  WALL_QUAT_WXYZ={WALL_QUAT_WXYZ} "
            f"approach_dir={np.round(base_approach_dir, 4).tolist()} "
            f"elevation={base_elevation_deg:+.1f}deg  "
            f"variants={len(self.grasp_quat_variants())}")
        self.get_logger().info(
            f"  LEFTMOST horizontal fallback x_corr="
            f"{LEFTMOST_GRASP_X_CORR_M*1000:+.0f}mm "
            f"offsets_mm={[round(v*1000) for v in LEFTMOST_GRASP_RETRY_OFFSETS]} "
            f"extra_request={self._leftmost_extra_advance_request_m*1000:.0f}mm "
            f"wall_margin={self._leftmost_wall_safety_margin_m*1000:.0f}mm "
            f"wall_override={self._leftmost_allow_wall_model_override} "
            f"pre_approach={PRE_APPROACH_OFFSET*1000:.0f}mm")
        if self._measured_tcp_model:
            self.get_logger().warn(
                "  TOOL_MODEL=measured_tcp_260mm: cuRobo ee_link is the measured "
                "grasp center; legacy length compensation and default extra advance "
                f"are disabled. plan_only={self._measured_tcp_plan_only}.")
        else:
            self.get_logger().warn(
                "  TOOL_GEOMETRY_LEGACY: planner offset="
                f"{LEGACY_EE_TO_TCP_OFFSET_M*1000:.0f}mm, measured grasp center="
                f"{MEASURED_FLANGE_TO_GRASP_CENTER_M*1000:.0f}mm "
                f"(model shortfall={TCP_MODEL_SHORTFALL_M*1000:.0f}mm).")
        if os.path.exists(ENVIRONMENT_YAML):
            self.get_logger().info(f"  environment loaded: {ENVIRONMENT_YAML}")
        self.get_logger().info(
            f"  marker place: enabled={self._enable_marker_place} "
            f"release={self._execute_marker_place_release} "
            f"allow_unverified_grasp={self._allow_unverified_grasp_place} "
            f"max_age={self._marker_place_max_age_sec:.0f}s")

        # 노드 시작 시 그리퍼를 approach 위치로 초기화 (2s 후 — gripper_service_node 연결 여유)
        self._gripper_init_done = False
        self.create_timer(2.0, self._init_gripper_once)

    def _init_gripper_once(self):
        if not self._gripper_init_done:
            self._gripper_init_done = True
            self._reset_gripper()

    def _reset_gripper(self):
        """파지 완료/실패 후 그리퍼를 approach 위치(GRIPPER_APPROACH_POS)로 복귀."""
        msg = Int32()
        msg.data = GRIPPER_APPROACH_POS
        self.gripper_pos_pub.publish(msg)

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def joint_state_cb(self, msg: JointState):
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        joints = [jmap.get(n) for n in self.JOINT_NAMES]
        if None not in joints:
            self.current_joints = joints

    def target_pose_cb(self, msg: PoseStamped):
        if self.current_joints is None:
            self.get_logger().warn("No joint state yet")
            return
        p, o = msg.pose.position, msg.pose.orientation
        ret = self.plan(self.current_joints, [p.x, p.y, p.z], [o.w, o.x, o.y, o.z])
        if ret is not None:
            self.execute_spline(*ret)

    def obstacles_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            cuboids = []
            for obj in data:
                cuboids.append(Cuboid(
                    name=obj["name"],
                    pose=[*obj["pos"], 1, 0, 0, 0],
                    dims=obj.get("dims", [0.05, 0.05, 0.05])
                ))
            self.dynamic_cuboids = cuboids
            self.update_curobo_world("dynamic obstacles")
        except Exception as e:
            self.get_logger().error(f"obstacles_cb error: {e}")

    # ── World 관리 ─────────────────────────────────────────────────────────────

    def update_curobo_world(self, reason="manual"):
        cuboids = self.static_cuboids + self.dynamic_cuboids
        self.motion_gen.update_world(WorldConfig(cuboid=cuboids, sphere=self.neighbor_spheres))
        self.get_logger().info(
            f"World updated ({reason}): static={len(self.static_cuboids)} "
            f"dynamic={len(self.dynamic_cuboids)} "
            f"neighbor_spheres={len(self.neighbor_spheres)}")
        self.runtime_log.log(
            "collision_world_update",
            reason=reason,
            cuboids=[{"name": c.name, "pose": c.pose, "dims": c.dims} for c in cuboids],
            neighbor_spheres=[
                {"name": s.name, "pose": s.pose, "radius": s.radius}
                for s in self.neighbor_spheres
            ],
        )

    def _scene_cb(self, msg: Float64MultiArray) -> None:
        data = msg.data
        self._scene_positions = [
            np.array([data[i], data[i+1], data[i+2]])
            for i in range(0, len(data) - 2, 3)
        ]
        self.runtime_log.log("scene_positions_received", positions_m=self._scene_positions)

    def _scan_status_cb(self, msg: String) -> None:
        self.runtime_log.log("scan_status", text=msg.data)

    def _cell_state_cb(self, msg: String) -> None:
        self.runtime_log.log("cell_state", text=msg.data)

    def _register_neighbor_obstacles(self, target_pos: np.ndarray) -> None:
        spheres = []
        registered_positions = []
        for i, pos in enumerate(self._scene_positions):
            if np.linalg.norm(pos - target_pos) < 0.035:
                continue
            spheres.append(Sphere(
                name=f"neighbor_{i}",
                pose=[float(pos[0]), float(pos[1]), float(pos[2]), 1.0, 0.0, 0.0, 0.0],
                radius=NEIGHBOR_SPHERE_RADIUS_M,
            ))
            registered_positions.append(np.array(pos, dtype=float))
        self.neighbor_spheres = spheres
        self._registered_neighbor_positions = registered_positions
        self.update_curobo_world("neighbor obstacles registered")
        self.get_logger().info(f"Registered {len(spheres)} neighbor sphere obstacle(s)")

    def _clear_neighbor_obstacles(self) -> None:
        had_neighbors = bool(self.neighbor_spheres or self._registered_neighbor_positions)
        self.neighbor_spheres = []
        self._registered_neighbor_positions = []
        if had_neighbors:
            self.update_curobo_world("neighbor obstacles cleared")

    # ── 충돌 진단 ──────────────────────────────────────────────────────────────

    def _check_state_feasible_with_world(self, joints, cuboids):
        try:
            self.motion_gen.update_world(WorldConfig(cuboid=cuboids))
            state = CuroboJointState.from_position(
                position=torch.tensor([self._clamp_joints(joints)], device="cuda:0", dtype=torch.float32),
                joint_names=self.JOINT_NAMES,
            )
            valid, status = self.motion_gen.check_start_state(state)
            return bool(valid), status
        finally:
            try:
                self.motion_gen.rollout_fn.primitive_collision_constraint.enable_cost()
                self.motion_gen.rollout_fn.robot_self_collision_constraint.enable_cost()
            except Exception:
                pass

    def diagnose_start_world_collision(self, joints, label):
        if not DEBUG_START_COLLISION:
            return
        full_world = self.static_cuboids + self.dynamic_cuboids
        far_dummy = Cuboid(
            name="debug_far_dummy",
            pose=[10.0, 10.0, 10.0, 1.0, 0.0, 0.0, 0.0],
            dims=[0.01, 0.01, 0.01],
        )
        tests = [("empty_world", [far_dummy])]
        tests += [(f"static:{c.name}", [c]) for c in self.static_cuboids]
        tests += [(f"dynamic:{c.name}", [c]) for c in self.dynamic_cuboids]
        bad = []
        try:
            for name, cuboids in tests:
                feasible, status = self._check_state_feasible_with_world(joints, cuboids)
                self.get_logger().warn(
                    f"{label} collision diag {name}: "
                    f"{'OK' if feasible else 'COLLISION'} status={status}")
                if not feasible:
                    bad.append(f"{name}:{status}")
        except Exception as e:
            self.get_logger().warn(f"{label} collision diag failed: {e}")
        finally:
            self.motion_gen.update_world(WorldConfig(cuboid=full_world))
        if bad:
            self.get_logger().error(f"{label} start collision suspects: {bad}")
        else:
            self.get_logger().warn(f"{label} no single obstacle reproduced the collision")

    def diagnose_js_endpoint_collision(self, start_joints, target_joints, label):
        if not DEBUG_START_COLLISION:
            return
        self.get_logger().warn(f"{label} endpoint collision diagnostic")
        self.diagnose_start_world_collision(start_joints, f"{label} start")
        self.diagnose_start_world_collision(target_joints, f"{label} goal")

    # ── 유틸 ──────────────────────────────────────────────────────────────────

    def _clamp_joints(self, joints):
        return [float(np.clip(j, lo, hi)) for j, (lo, hi) in zip(joints, self.JOINT_LIMITS)]

    def grasp_candidates_for_target(self, straw):
        if self._measured_tcp_model:
            return [MEASURED_TCP_FINAL_STANDOFF_M]
        if straw[0] > 0.25:
            return [-0.03, 0.0]
        if straw[0] < -0.30:
            # 더 깊은 30/35mm부터 검사하되 cuRobo가 검증한 endpoint만 실행한다.
            return LEFTMOST_GRASP_RETRY_OFFSETS
        return GRASP_RETRY_OFFSETS

    def grasp_quat_variants(self):
        if self._measured_tcp_model:
            return MEASURED_TCP_GRASP_QUAT_RETRY_VARIANTS
        return GRASP_QUAT_RETRY_VARIANTS

    def call_trigger(self, client):
        if not client.wait_for_service(timeout_sec=3.0):
            return
        future = client.call_async(Trigger.Request())
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 10.0:
            time.sleep(0.1)

    def _verify_grasp(self):
        """Position/current feedback로 자동 접촉 판정. 사람 라벨은 정확도 검증에 유지한다."""
        if not self.cli_gripper_read_state.wait_for_service(timeout_sec=0.5):
            return "GRASP_UNVERIFIED", -1, -1, "read_state service unavailable"
        future = self.cli_gripper_read_state.call_async(Trigger.Request())
        t0 = time.time()
        while not future.done() and (time.time() - t0) < GRASP_VERIFY_TIMEOUT_SEC:
            time.sleep(0.05)
        if not future.done():
            return "GRASP_UNVERIFIED", -1, -1, "read_state timeout"
        res = future.result()
        if not res or not res.success:
            return "GRASP_UNVERIFIED", -1, -1, "read_state service error"
        try:
            state = json.loads(res.message)
            position = int(state["position"])
            current_raw = int(state["current_raw"])
        except (ValueError, TypeError, KeyError, AttributeError, json.JSONDecodeError):
            return "GRASP_UNVERIFIED", -1, -1, f"parse error: {res.message!r}"
        if position < 0 or current_raw < 0:
            return (
                "GRASP_UNVERIFIED", position, current_raw,
                "hardware state read failed (virtual mode or serial error)")
        if position >= GRASP_EMPTY_POSITION_THRESHOLD:
            return "GRASP_EMPTY", position, current_raw, (
                f"fully closed (pos={position} >= threshold={GRASP_EMPTY_POSITION_THRESHOLD})")
        if (
            self._grasp_current_contact_threshold_raw >= 0
            and current_raw < self._grasp_current_contact_threshold_raw
        ):
            return "GRASP_UNVERIFIED", position, current_raw, (
                f"position indicates contact but current={current_raw} below calibrated "
                f"threshold={self._grasp_current_contact_threshold_raw}")
        return "GRASP_CONTACT_DETECTED", position, current_raw, (
            f"jaw stopped at pos={position} and current_raw={current_raw}; "
            f"current threshold={'disabled' if self._grasp_current_contact_threshold_raw < 0 else self._grasp_current_contact_threshold_raw}")

    # ── 플래닝 ────────────────────────────────────────────────────────────────

    def trajectory_in_operational_limits(self, traj_rad, label):
        traj_deg = np.rad2deg(traj_rad)
        for joint_idx, (lo, hi) in enumerate(OPERATIONAL_JOINT_LIMITS_DEG):
            vals = traj_deg[:, joint_idx]
            if np.any(vals < lo) or np.any(vals > hi):
                self.get_logger().warn(
                    f"{label} rejected: J{joint_idx+1} out of [{lo:.0f}°, {hi:.0f}°] "
                    f"(range {vals.min():.1f}°..{vals.max():.1f}°)")
                return False
        return True

    def trajectory_has_reasonable_swing(self, traj_rad, start_joints, label):
        traj_deg = np.rad2deg(traj_rad)
        start_deg = np.rad2deg(start_joints)
        for joint_idx, max_delta in enumerate(MAX_HARVEST_JOINT_DELTA_DEG):
            vals = traj_deg[:, joint_idx]
            if joint_idx in WRAP_EQUIVALENT_JOINT_IDX:
                # endpoint 등가 거리가 아닌 trajectory 실제 range 검사:
                # normalize가 "돌아가는 방향"을 따라붙어도 310° 스윙을 탐지
                delta_vals = np.abs(vals - float(vals[0]))
            else:
                delta_vals = np.abs(vals - start_deg[joint_idx])
            delta = float(np.max(delta_vals))
            if delta > max_delta:
                end_deg = traj_deg[-1, joint_idx]
                self.get_logger().warn(
                    f"{label} rejected: J{joint_idx+1} swing {delta:.1f}° > {max_delta:.1f}° "
                    f"(start={start_deg[joint_idx]:.1f}° → end={end_deg:.1f}°)")
                return False
        return True

    def normalize_trajectory_equivalents(self, traj_rad, label):
        traj_deg = np.rad2deg(traj_rad).astype(float)
        rewritten = []
        for joint_idx in WRAP_EQUIVALENT_JOINT_IDX:
            lo, hi = OPERATIONAL_JOINT_LIMITS_DEG[joint_idx]
            original = traj_deg[:, joint_idx].copy()
            prev = None
            for row_idx, value in enumerate(original):
                candidates = [value + 360.0 * k for k in range(-2, 3)]
                valid = [c for c in candidates if lo <= c <= hi]
                if not valid:
                    continue
                reference = prev if prev is not None else value
                best = min(valid, key=lambda c: abs(c - reference))
                traj_deg[row_idx, joint_idx] = best
                prev = best
            if np.max(np.abs(traj_deg[:, joint_idx] - original)) > 1e-6:
                rewritten.append(
                    f"J{joint_idx+1} {float(np.min(original)):.1f}~{float(np.max(original)):.1f}"
                    f" -> {float(np.min(traj_deg[:, joint_idx])):.1f}~{float(np.max(traj_deg[:, joint_idx])):.1f}"
                )
        if rewritten:
            self.get_logger().info(
                f"{label} joint equivalent rewrite: " + "; ".join(rewritten))
        return np.deg2rad(traj_deg)

    def trajectory_has_no_spline_jumps(self, traj_rad, label, max_jump_deg=270.0):
        """normalize 후 연속 waypoint 간 대형 각도 점프 검사.

        J4/J6가 ±한계 경계를 넘으면 normalize가 강제로 반대 부호로 바꾸면서
        직전 waypoint와 357° 차이가 생기고 Doosan 스플라인이 360° 스핀함.
        이를 실행 전에 탐지해서 plan 자체를 reject.
        """
        traj_deg = np.rad2deg(traj_rad)
        for joint_idx in WRAP_EQUIVALENT_JOINT_IDX:
            diffs = np.abs(np.diff(traj_deg[:, joint_idx]))
            if len(diffs) == 0:
                continue
            max_diff = float(np.max(diffs))
            if max_diff > max_jump_deg:
                bad_idx = int(np.argmax(diffs))
                self.get_logger().warn(
                    f"{label} rejected: J{joint_idx+1} spline jump {max_diff:.1f}° "
                    f"> {max_jump_deg:.1f}° at waypoint {bad_idx} "
                    f"(limit boundary crossing — normalize 불연속)")
                return False
        return True

    def plan(self, start_joints, target_pos, target_quat_wxyz, num_ik_seeds=32,
             max_attempts=None, timeout_sec=None):
        t0 = time.time()
        start_joints = self._clamp_joints(start_joints)
        start_state = CuroboJointState.from_position(
            position=torch.tensor([start_joints], device="cuda:0", dtype=torch.float32),
            joint_names=self.JOINT_NAMES,
        )
        target_pose = Pose(
            position=torch.tensor([target_pos], device="cuda:0", dtype=torch.float32),
            quaternion=torch.tensor([target_quat_wxyz], device="cuda:0", dtype=torch.float32),
        )
        result = self.motion_gen.plan_single(
            start_state, target_pose,
            MotionGenPlanConfig(
                num_ik_seeds=num_ik_seeds,
                max_attempts=(
                    max_attempts
                    if max_attempts is not None
                    else CARTESIAN_PLAN_MAX_ATTEMPTS
                ),
                timeout=(
                    timeout_sec
                    if timeout_sec is not None
                    else CARTESIAN_PLAN_TIMEOUT_SEC
                ),
                enable_graph_attempt=None,
            ),
        )
        dt = (time.time() - t0) * 1000

        if result.success.item():
            traj = result.get_interpolated_plan().position.cpu().numpy()
            traj = self.normalize_trajectory_equivalents(traj, "Cartesian plan")
            if not self.trajectory_in_operational_limits(traj, "Cartesian plan"):
                self.runtime_log.log(
                    "curobo_plan_rejected",
                    planner="cartesian",
                    reason="operational_joint_limits",
                    start_joints_rad=start_joints,
                    target_pos_m=target_pos,
                    target_quat_wxyz=target_quat_wxyz,
                    trajectory_rad=traj,
                )
                return None
            if not self.trajectory_has_no_spline_jumps(traj, "Cartesian plan"):
                self.runtime_log.log(
                    "curobo_plan_rejected",
                    planner="cartesian",
                    reason="spline_jump",
                    start_joints_rad=start_joints,
                    target_pos_m=target_pos,
                    target_quat_wxyz=target_quat_wxyz,
                    trajectory_rad=traj,
                )
                return None
            if not self.trajectory_has_reasonable_swing(traj, start_joints, "Cartesian plan"):
                self.runtime_log.log(
                    "curobo_plan_rejected",
                    planner="cartesian",
                    reason="joint_swing",
                    start_joints_rad=start_joints,
                    target_pos_m=target_pos,
                    target_quat_wxyz=target_quat_wxyz,
                    trajectory_rad=traj,
                )
                return None
            motion_time = float(result.motion_time.item())
            end_deg = [f"{np.rad2deg(v):.1f}" for v in traj[-1]]
            self.get_logger().info(
                f"Plan OK {dt:.0f}ms {traj.shape[0]}pts {motion_time:.2f}s | "
                f"goal={[f'{v*1000:.0f}' for v in target_pos]}mm | "
                f"end_J=[{', '.join(end_deg)}]°")
            self.runtime_log.log(
                "curobo_plan_success",
                planner="cartesian",
                planning_latency_ms=dt,
                motion_time_sec=motion_time,
                start_joints_rad=start_joints,
                target_pos_m=target_pos,
                target_quat_wxyz=target_quat_wxyz,
                trajectory_rad=traj,
            )
            return traj, motion_time
        else:
            status = str(getattr(result, "status", "UNKNOWN"))
            start_deg = [f"{np.rad2deg(v):.1f}" for v in start_joints]
            self.get_logger().error(
                f"Plan FAIL {dt:.0f}ms | status={status} | "
                f"goal={[f'{v*1000:.0f}' for v in target_pos]}mm | "
                f"start_J=[{', '.join(start_deg)}]°")
            if "INVALID_START_STATE_WORLD_COLLISION" in status:
                self.diagnose_start_world_collision(start_joints, "Cartesian plan")
            self.runtime_log.log(
                "curobo_plan_fail",
                planner="cartesian",
                status=status,
                planning_latency_ms=dt,
                start_joints_rad=start_joints,
                target_pos_m=target_pos,
                target_quat_wxyz=target_quat_wxyz,
            )
            return None

    def plan_js(self, start_joints, target_joints_rad, label, skip_swing_check=False):
        t0 = time.time()
        start_joints = self._clamp_joints(start_joints)
        target_joints_rad = self._clamp_joints(target_joints_rad)
        start_state = CuroboJointState.from_position(
            position=torch.tensor([start_joints], device="cuda:0", dtype=torch.float32),
            joint_names=self.JOINT_NAMES,
        )
        goal_state = CuroboJointState.from_position(
            position=torch.tensor([target_joints_rad], device="cuda:0", dtype=torch.float32),
            joint_names=self.JOINT_NAMES,
        )
        result = self.motion_gen.plan_single_js(
            start_state, goal_state, MotionGenPlanConfig(enable_graph=True)
        )
        dt = (time.time() - t0) * 1000

        if result.success.item():
            traj = result.get_interpolated_plan().position.cpu().numpy()
            traj = self.normalize_trajectory_equivalents(traj, label)
            if not self.trajectory_in_operational_limits(traj, label):
                self.runtime_log.log(
                    "curobo_plan_rejected", planner="joint_space", label=label,
                    reason="operational_joint_limits", trajectory_rad=traj)
                return None
            if not skip_swing_check and not self.trajectory_has_reasonable_swing(traj, start_joints, label):
                self.runtime_log.log(
                    "curobo_plan_rejected", planner="joint_space", label=label,
                    reason="joint_swing", trajectory_rad=traj)
                return None
            motion_time = float(result.motion_time.item())
            self.get_logger().info(
                f"{label} JS Plan OK {dt:.0f}ms {traj.shape[0]}pts {motion_time:.2f}s | "
                f"goal={[f'{v:.1f}' for v in np.rad2deg(target_joints_rad)]}°")
            self.runtime_log.log(
                "curobo_plan_success",
                planner="joint_space",
                label=label,
                planning_latency_ms=dt,
                motion_time_sec=motion_time,
                start_joints_rad=start_joints,
                target_joints_rad=target_joints_rad,
                trajectory_rad=traj,
            )
            return traj, motion_time

        status = getattr(result, "status", "?")
        self.get_logger().error(
            f"{label} JS Plan FAIL {dt:.0f}ms | status={status} | "
            f"goal={[f'{v:.1f}' for v in np.rad2deg(target_joints_rad)]}°")
        if "INVALID_START_STATE_WORLD_COLLISION" in str(status) or "GRAPH_FAIL" in str(status):
            self.diagnose_js_endpoint_collision(start_joints, target_joints_rad, label)
        self.runtime_log.log(
            "curobo_plan_fail",
            planner="joint_space",
            label=label,
            status=str(status),
            planning_latency_ms=dt,
            start_joints_rad=start_joints,
            target_joints_rad=target_joints_rad,
        )
        return None

    def execute_spline(self, traj_rad, motion_time: float) -> bool:
        if not self.cli_spline.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveSplineJoint not available")
            return False
        traj_deg = np.rad2deg(traj_rad)
        n = traj_deg.shape[0]
        if n > MAX_SPLINE_POINTS:
            idx = np.linspace(0, n - 1, MAX_SPLINE_POINTS, dtype=int)
            traj_deg = traj_deg[idx]
            n = MAX_SPLINE_POINTS

        from std_msgs.msg import Float64MultiArray as F64MA
        req = MoveSplineJoint.Request()
        req.pos_cnt = n
        for row in traj_deg:
            pt = F64MA()
            pt.data = row.tolist()
            req.pos.append(pt)
        req.vel = [120.0] * 6
        req.acc = [180.0] * 6
        req.time = max(float(motion_time) * SPLINE_TIME_SCALE, SPLINE_MIN_TIME)
        req.mode = 0
        req.sync_type = 0

        self.get_logger().info(
            f"Spline {n}pts plan={motion_time:.2f}s exec={req.time:.2f}s "
            f"→ end={[f'{v:.1f}' for v in traj_deg[-1]]}°")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_spline_joint",
            service="/dsr01/motion/move_spline_joint",
            trajectory_deg=traj_deg,
            planned_motion_time_sec=motion_time,
            requested_time_sec=req.time,
            velocity_deg_s=req.vel,
            acceleration_deg_s2=req.acc,
        )
        future = self.cli_spline.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)

        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error("Spline failed/timeout")
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_spline_joint",
            success=bool(ok),
            current_joints_rad=self.current_joints,
        )
        return ok

    def execute_tool_z_line(self, distance_m: float, motion_label="FINAL_APPROACH_STRAIGHT",
                            vel_mm_s: float = None, acc_mm_s2: float = None,
                            min_distance_m: float = 0.02) -> bool:
        """현재 TCP 자세를 유지하고 TOOL Z축 방향으로 직선 이동."""
        if not min_distance_m <= abs(distance_m) <= 0.25:
            self.get_logger().error(
                f"MoveLine rejected: {motion_label} distance={distance_m*1000:.1f}mm "
                f"allowed={min_distance_m*1000:.1f}..250.0mm")
            return False
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveLine not available")
            return False

        vel = vel_mm_s if vel_mm_s is not None else FINAL_APPROACH_VEL_MM_S
        acc = acc_mm_s2 if acc_mm_s2 is not None else FINAL_APPROACH_ACC_MM_S2

        req = MoveLine.Request()
        req.pos = [0.0, 0.0, float(distance_m * 1000.0), 0.0, 0.0, 0.0]
        req.vel = [vel, 10.0]
        req.acc = [acc, 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 1         # DR_TOOL
        req.mode = 1        # DR_MV_MOD_REL
        req.blend_type = 0
        req.sync_type = 0   # SYNC: 완전히 도착한 뒤 응답

        self.get_logger().info(
            f"{motion_label} TOOL {'+Z' if distance_m > 0 else '-Z'} "
            f"{abs(distance_m)*1000:.1f}mm "
            f"vel={vel:.1f}mm/s")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_line",
            label=motion_label,
            service="/dsr01/motion/move_line",
            reference_frame="tool",
            relative_pose_mm_deg=req.pos,
            velocity=req.vel,
            acceleration=req.acc,
        )
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 30.0:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error(f"{motion_label} MoveLine failed/timeout")
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_line",
            label=motion_label,
            success=bool(ok),
            current_joints_rad=self.current_joints,
        )
        return ok

    def _execute_pitch_detach(self) -> bool:
        """파지 후 TCP를 BASE -Z 방향으로 당겨 줄기 분리. 회전 없음."""
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("DETACH_PULL: MoveLine service unavailable")
            return False
        req = MoveLine.Request()
        req.pos = [0.0, 0.0, -float(DETACH_PULL_DOWN_MM), 0.0, 0.0, 0.0]
        req.vel = [float(DETACH_PULL_VEL_MM_S), 10.0]
        req.acc = [30.0, 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0   # BASE frame
        req.mode = 1  # RELATIVE
        req.blend_type = 0
        req.sync_type = 0
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 30.0:
            time.sleep(0.05)
        ok = future.done() and bool(future.result() and future.result().success)
        self.get_logger().info(
            f"DETACH_PULL_DOWN: BASE -Z {DETACH_PULL_DOWN_MM:.0f}mm "
            f"→ {'OK' if ok else 'FAIL'}")
        self.runtime_log.log("detach_pull_down",
                             pull_mm=DETACH_PULL_DOWN_MM,
                             vel_mm_s=DETACH_PULL_VEL_MM_S, success=ok)
        return ok

    def execute_base_z_relative(self, distance_m: float, motion_label: str,
                                vel_mm_s: float = 30.0) -> bool:
        """BASE 기준 Z축 상대 직선 이동. 크레인 접근/이탈 하강·상승에 사용."""
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"{motion_label}: MoveLine service unavailable")
            return False
        req = MoveLine.Request()
        req.pos = [0.0, 0.0, float(distance_m * 1000.0), 0.0, 0.0, 0.0]
        req.vel = [float(vel_mm_s), 10.0]
        req.acc = [30.0, 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0   # DR_BASE
        req.mode = 1  # DR_MV_MOD_REL
        req.blend_type = 0
        req.sync_type = 0
        direction = "+Z" if distance_m > 0 else "-Z"
        self.get_logger().info(
            f"{motion_label} BASE {direction} {abs(distance_m)*1000:.0f}mm "
            f"vel={vel_mm_s:.0f}mm/s")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_line",
            label=motion_label,
            service="/dsr01/motion/move_line",
            reference_frame="base",
            relative_pose_mm_deg=req.pos,
            velocity=req.vel,
        )
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 30.0:
            time.sleep(0.05)
        ok = future.done() and bool(future.result() and future.result().success)
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_line",
            label=motion_label,
            success=bool(ok),
        )
        if not ok:
            self.get_logger().error(f"{motion_label} BASE {direction} failed/timeout")
        return ok

    def execute_base_line(self, posx_mm_deg, motion_label, vel_mm_s=20.0) -> bool:
        """베이스 기준 절대 TCP 직선 이동. Marker place의 수직 above/release에만 사용."""
        if len(posx_mm_deg) != 6:
            self.get_logger().error(f"{motion_label}: expected 6D posx")
            return False
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveLine not available")
            return False

        req = MoveLine.Request()
        req.pos = [float(v) for v in posx_mm_deg]
        req.vel = [float(vel_mm_s), 10.0]
        req.acc = [30.0, 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0         # DR_BASE
        req.mode = 0        # DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 0

        self.get_logger().info(
            f"{motion_label} BASE ABS "
            f"xyz={[round(v, 1) for v in req.pos[:3]]}mm "
            f"abc={[round(v, 1) for v in req.pos[3:]]}deg")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_line",
            label=motion_label,
            service="/dsr01/motion/move_line",
            reference_frame="base",
            absolute_pose_mm_deg=req.pos,
            velocity=req.vel,
            acceleration=req.acc,
        )
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error(f"{motion_label} MoveLine failed/timeout")
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_line",
            label=motion_label,
            success=bool(ok),
            current_joints_rad=self.current_joints,
        )
        return ok

    def _doosan_zyz_to_wxyz(self, rx_deg: float, ry_deg: float, rz_deg: float):
        """Doosan ZYZ Euler (deg) → quaternion [w, x, y, z] for cuRobo."""
        r = SciR.from_euler("ZYZ", [rx_deg, ry_deg, rz_deg], degrees=True)
        xyzw = r.as_quat()
        return [float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])]

    def _nearest_equivalent_joints(self, base_joints_deg):
        """J4/J6를 현재 위치에서 가장 가까운 360° equivalent로 조정."""
        if self.current_joints is None:
            return base_joints_deg
        current_deg = np.rad2deg(self.current_joints)
        joints = list(base_joints_deg)
        for i in WRAP_EQUIVALENT_JOINT_IDX:
            lo, hi = OPERATIONAL_JOINT_LIMITS_DEG[i]
            candidates = [joints[i] + 360.0 * k for k in range(-2, 3)]
            valid = [c for c in candidates if lo <= c <= hi]
            if valid:
                joints[i] = min(valid, key=lambda c: abs(c - current_deg[i]))
        return joints

    def home_joints_near_current(self):
        return self._nearest_equivalent_joints(HOME_JOINTS_DEG)

    def overview_joints_near_current(self):
        return self._nearest_equivalent_joints(OVERVIEW_JOINTS_DEG)

    def movej_direct(self, joints_deg, vel=40.0, acc=60.0):
        """cuRobo 우회 — Doosan MoveJoint 직접 호출. 최후 수단용."""
        if not self.cli_movej.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveJoint service not available")
            return False
        req = MoveJoint.Request()
        req.pos = [float(v) for v in joints_deg]
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        self.get_logger().warn(
            f"MoveJoint direct → {[round(v, 1) for v in joints_deg]}° vel={vel}")
        future = self.cli_movej.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 90.0:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error("MoveJoint direct failed/timeout")
        return ok

    def plan_to_fixed_joints_pose(self, start_joints, target_joints_deg, label,
                                   skip_swing_check=False):
        """고정 joint 자세 이동 — cuRobo joint-space plan."""
        target_joints_rad = np.deg2rad(target_joints_deg).tolist()
        ret = self.plan_js(start_joints, target_joints_rad, label,
                           skip_swing_check=skip_swing_check)
        if ret is not None and self.execute_spline(*ret):
            return True, ret[0][-1].tolist()
        self.get_logger().warn(f"{label} CuRobo joint-space failed")
        return False, start_joints

    def _latest_tray_cells_json(self):
        if self._tray_cells_json:
            return self._tray_cells_json
        files = sorted(
            glob.glob(DEFAULT_TRAY_CELLS_GLOB),
            key=os.path.getmtime,
            reverse=True,
        )
        return files[0] if files else None

    def _load_marker_place_target(self):
        path = self._latest_tray_cells_json()
        if not path or not os.path.isfile(path):
            self.get_logger().error("MARKER_PLACE_BLOCKED: tray cells JSON not found")
            return None
        age_sec = time.time() - os.path.getmtime(path)
        if age_sec > self._marker_place_max_age_sec:
            self.get_logger().error(
                f"MARKER_PLACE_BLOCKED: tray localization stale "
                f"age={age_sec:.0f}s > {self._marker_place_max_age_sec:.0f}s")
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            cells = data.get("cells", [])
            if not cells:
                raise ValueError("no cells")
            cell = cells[self._marker_place_slot_idx % len(cells)]
            tcp = cell["position_tcp_mm"]
            orient = cell["task_orientation_deg"]
            release = [
                float(tcp["x"]), float(tcp["y"]), float(tcp["z"]),
                float(orient["rx"]), float(orient["ry"]), float(orient["rz"]),
            ]
            if not (
                -800.0 <= release[0] <= 800.0
                and -800.0 <= release[1] <= 800.0
                and 250.0 <= release[2] <= 1200.0
            ):
                raise ValueError(f"target outside guarded workspace: {release[:3]}")
        except Exception as exc:
            self.get_logger().error(f"MARKER_PLACE_BLOCKED: invalid tray JSON ({exc})")
            return None

        above = list(release)
        above[2] += self._marker_place_above_clearance_m * 1000.0
        gripper_offset = data.get("gripper_offset") or {}
        self.runtime_log.log(
            "marker_place_target_loaded",
            path=path,
            age_sec=age_sec,
            slot_index=cell.get("index"),
            row=cell.get("row"),
            col=cell.get("col"),
            release_posx_mm_deg=release,
            above_posx_mm_deg=above,
            source_standoff_mm=gripper_offset.get("fingertip_standoff_mm"),
        )
        return {
            "path": path,
            "slot_index": int(cell.get("index", self._marker_place_slot_idx)),
            "release": release,
            "above": above,
        }

    def _execute_marker_place_after_retreat(self, retreat_joints):
        """Marker-derived place. Release 승인 전에는 above에서 정지한다."""
        target = self._load_marker_place_target()
        if target is None:
            return "skip", retreat_joints   # tray 없음/stale → soft skip, hold 없음

        self.get_logger().info(
            f"5 marker place slot={target['slot_index']} via overview/tray-view")
        overview_deg = self.overview_joints_near_current()
        ok, overview_joints = self.plan_to_fixed_joints_pose(
            retreat_joints, overview_deg, "marker place transfer overview",
            skip_swing_check=True)
        if not ok:
            self.get_logger().error(
                "MARKER_PLACE_BLOCKED: transfer overview plan failed; holding fruit")
            return "failed", retreat_joints

        tray_view_deg = self._nearest_equivalent_joints(TRAY_VIEW_JOINTS_DEG)
        ok, tray_view_joints = self.plan_to_fixed_joints_pose(
            overview_joints, tray_view_deg, "marker place tray view",
            skip_swing_check=True)
        if not ok:
            self.get_logger().error(
                "MARKER_PLACE_BLOCKED: tray-view plan failed; holding fruit")
            return "failed", overview_joints

        # ABOVE: cuRobo Cartesian plan (TRAY_VIEW_JOINTS ≈ ABOVE, trivial or minor move)
        above_pos_m = [v / 1000.0 for v in target["above"][:3]]
        above_quat = self._doosan_zyz_to_wxyz(*target["above"][3:])
        self.get_logger().info(
            f"MARKER_PLACE_ABOVE cuRobo "
            f"xyz={[round(v, 1) for v in target['above'][:3]]}mm "
            f"abc={[round(v, 1) for v in target['above'][3:]]}deg")
        above_plan = self.plan(tray_view_joints, above_pos_m, above_quat)
        if above_plan is None:
            self.get_logger().error(
                "MARKER_PLACE_BLOCKED: above cuRobo plan failed; holding fruit")
            return "failed", tray_view_joints
        ok_above = self.execute_spline(*above_plan)
        above_joints = list(
            above_plan[0][-1].tolist() if ok_above else tray_view_joints)
        if not ok_above:
            self.get_logger().error("MARKER_PLACE_BLOCKED: above spline exec failed; holding fruit")
            return "failed", tray_view_joints

        if not self._execute_marker_place_release:
            self.get_logger().warn(
                "MARKER_PLACE_PREVIEW_HOLD: above reached; release disabled. "
                "Inspect clearance before enabling execute_marker_place_release.")
            return "preview_hold", list(self.current_joints or above_joints)

        # RELEASE: cuRobo Cartesian plan — avoids kinematic flip caused by BASE ABS
        release_pos_m = [v / 1000.0 for v in target["release"][:3]]
        release_quat = self._doosan_zyz_to_wxyz(*target["release"][3:])
        self.get_logger().info(
            f"MARKER_PLACE_RELEASE_DESCEND cuRobo "
            f"xyz={[round(v, 1) for v in target['release'][:3]]}mm "
            f"abc={[round(v, 1) for v in target['release'][3:]]}deg")
        release_plan = self.plan(above_joints, release_pos_m, release_quat)
        if release_plan is None:
            self.get_logger().error(
                "MARKER_PLACE_BLOCKED: release cuRobo plan failed; holding fruit")
            return "failed", list(self.current_joints or above_joints)
        ok_release = self.execute_spline(*release_plan)
        if not ok_release:
            self.get_logger().error(
                "MARKER_PLACE_BLOCKED: release spline exec failed; holding fruit")
            return "failed", list(self.current_joints or above_joints)
        release_joints = list(release_plan[0][-1].tolist())

        self.get_logger().info("6 marker place release gripper")
        self.runtime_log.log(
            "gripper_command", command="release",
            slot_index=target["slot_index"])
        self.call_trigger(self.cli_gripper_open)
        time.sleep(2.0)

        # RETREAT: release pose에서 먼저 above로 상승한 뒤 tray-view로 복귀한다.
        # release에서 tray-view 관절 자세로 바로 이동하면 tray body를 가로지를 수 있다.
        above_retreat_plan = self.plan(release_joints, above_pos_m, above_quat)
        if above_retreat_plan is None or not self.execute_spline(*above_retreat_plan):
            self.get_logger().error(
                "MARKER_PLACE_RELEASED_BUT_ABOVE_RETREAT_FAILED: holding position")
            return "failed_after_release", list(self.current_joints or release_joints)
        above_retreat_joints = list(above_retreat_plan[0][-1].tolist())

        # Known tray-view configuration으로 collision-aware joint-space 복귀.
        tray_view_deg_retreat = self._nearest_equivalent_joints(TRAY_VIEW_JOINTS_DEG)
        ok_retreat, _ = self.plan_to_fixed_joints_pose(
            above_retreat_joints, tray_view_deg_retreat,
            "MARKER_PLACE_TRAY_VIEW_RETURN")
        if not ok_retreat:
            self.get_logger().error(
                "MARKER_PLACE_RELEASED_BUT_RETREAT_FAILED: holding position")
            return "failed_after_release", list(
                self.current_joints or above_retreat_joints)

        self._marker_place_slot_idx += 1
        self.runtime_log.log(
            "marker_place_complete",
            result_code="PLACE_SEQUENCE_COMPLETE_UNVERIFIED",
            slot_index=target["slot_index"],
            tray_cells_json=target["path"],
        )
        return "success", list(self.current_joints or tray_view_joints)

    # ── Pick 시퀀스 ────────────────────────────────────────────────────────────

    def _hold_pick_sequence(self, reason: str):
        self._sequence_hold_reason = reason
        self.runtime_log.log(
            "pick_sequence_hold_latched",
            reason=reason,
            current_joints_rad=self.current_joints,
        )
        self.get_logger().warn(
            f"PICK_SEQUENCE_HOLD_LATCHED reason={reason}; "
            "new pick targets are blocked until planner restart")

    def pick_pose_cb(self, msg: PoseStamped):
        if self._sequence_hold_reason is not None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec - self._last_sequence_hold_warn_sec >= 5.0:
                self._last_sequence_hold_warn_sec = now_sec
                self.get_logger().warn(
                    f"Pick target ignored: sequence hold "
                    f"({self._sequence_hold_reason})")
            return
        if self.current_joints is None:
            self.get_logger().warn("No joint state yet")
            return
        if self._pick_busy:
            self.get_logger().warn("Pick already in progress — ignored")
            return
        self._pick_busy = True
        try:
            self._pick(msg)
        finally:
            self._pick_busy = False

    def _pick(self, msg: PoseStamped):
        p = msg.pose.position
        # 같은 셀의 다음 target을 계속 처리할 수 있도록 이번 pick이 시작된
        # taught scan pose를 저장한다. overview 복귀는 scan_executor가 담당한다.
        pick_start_joints = list(self.current_joints)
        self.runtime_log.log(
            "pick_sequence_start",
            input_frame=msg.header.frame_id,
            input_target_m=[p.x, p.y, p.z],
            input_quat_xyzw=[
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ],
            start_joints_rad=pick_start_joints,
        )

        # Y 클램핑: berry는 벽 표면보다 뒤에 있을 수 없음 (FK drift 보정)
        detection_raw_y = float(p.y)   # 클램핑 전 원본값 — measured TCP 적응형 접근 거리 계산용
        raw_y = detection_raw_y
        wall_y_clamped = raw_y > WALL_SURFACE_Y_M
        if wall_y_clamped:
            self.get_logger().warn(
                f"Detection Y={raw_y*1000:.0f}mm > wall surface {WALL_SURFACE_Y_M*1000:.0f}mm "
                f"(FK calibration drift) — clamped to {WALL_SURFACE_Y_M*1000:.0f}mm")
            raw_y = WALL_SURFACE_Y_M
        raw_straw = np.array([p.x, raw_y, max(p.z, 0.05)])
        straw = raw_straw + np.array([0.0, 0.0, GRASP_Z_BIAS])
        straw[2] = max(straw[2], 0.05)

        x_min, x_max = DIRECT_GRASP_TARGET_X_RANGE_M
        if not (x_min <= float(raw_straw[0]) <= x_max):
            self.get_logger().warn(
                f"ABORT: pick target x={raw_straw[0]*1000:.0f}mm outside "
                f"[{x_min*1000:.0f}, {x_max*1000:.0f}]mm")
            self.pick_complete_pub.publish(Empty())
            return

        grasp_retry_offsets = self.grasp_candidates_for_target(straw)

        self.get_logger().info(
            f"=== PICK 딸기 raw=({raw_straw[0]*1000:.0f},{raw_straw[1]*1000:.0f},{raw_straw[2]*1000:.0f})mm "
            f"grasp=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
            f"z_bias={GRASP_Z_BIAS*1000:+.0f}mm ===")
        self.runtime_log.log(
            "pick_target_prepared",
            raw_target_m=raw_straw,
            grasp_target_m=straw,
            grasp_z_bias_m=GRASP_Z_BIAS,
            wall_y_clamped=wall_y_clamped,
        )

        self._register_neighbor_obstacles(straw)
        self.motion_gen.detach_object_from_robot()

        if raw_straw[0] < -0.30 and not self._measured_tcp_model:
            straw[0] += LEFTMOST_GRASP_X_CORR_M

        # 2. Grasp (cuRobo 2-step): 6cm pre-approach → 직선 진입
        # 직전 측방 편차가 줄기 형상/검출점 영향인지 분리하기 위해 6cm를 재검증한다.
        grasp_quat_variants = self.grasp_quat_variants()
        n_offsets = len(grasp_retry_offsets)
        n_quats   = len(grasp_quat_variants)
        self.get_logger().info(
            f"2 grasp (CuRobo 2-step {PRE_APPROACH_OFFSET*100:.0f}cm pre) — "
            f"trying {n_offsets} offsets × {n_quats} quats "
            f"| target=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
            f"| start_J1={np.rad2deg(self.current_joints[0]):.1f}°")
        ret_pre   = None
        ret_grasp = None
        used_grasp_offset = None
        used_grasp_variant = None
        used_approach_dir = None
        used_grasp_ee_pos = None
        grasp_attempt = 0
        for quat_frame, axis, quat_deg in grasp_quat_variants:
            q_delta = quat_from_axis_angle(axis, np.deg2rad(quat_deg))
            if quat_frame == "base":
                q_retry = quat_multiply_wxyz(q_delta, WALL_QUAT_WXYZ)
            else:
                q_retry = quat_multiply_wxyz(WALL_QUAT_WXYZ, q_delta)
            approach_dir = np.array(quat_rotate_vec(q_retry, [0.0, 0.0, 1.0]))
            ee_pre = straw - (
                PRE_APPROACH_OFFSET + self._ee_to_tcp_offset_m
            ) * approach_dir
            if self._measured_tcp_model and CRANE_Z_OFFSET_M > 0:
                # 크레인 접근: cuRobo는 실제 파지 높이보다 CRANE_Z_OFFSET_M 위로 계획
                # BASE -Z 하강으로 실제 높이에 도달 후 TOOL +Z 진입
                ee_pre = ee_pre + np.array([0.0, 0.0, CRANE_Z_OFFSET_M])
            r_pre_for_variant = self.plan(
                self.current_joints, ee_pre.tolist(), q_retry, num_ik_seeds=48
            )
            if r_pre_for_variant is None:
                grasp_attempt += len(grasp_retry_offsets)
                continue
            pre_joints = r_pre_for_variant[0][-1].tolist()

            # The measured TCP model intentionally validates the long move only.
            # cuRobo repeatedly rejects the final 30~60mm near the wall even
            # though the taught baseline executes that segment as a guarded,
            # straight Doosan MoveLine. Keep the TCP at least 30mm from the
            # modeled wall and validate the final segment by distance gate.
            if self._measured_tcp_model:
                ret_pre = r_pre_for_variant
                ret_grasp = r_pre_for_variant
                used_grasp_offset = MEASURED_TCP_FINAL_STANDOFF_M
                used_grasp_variant = (quat_frame, axis, quat_deg)
                used_approach_dir = approach_dir
                used_grasp_ee_pos = (
                    straw - MEASURED_TCP_FINAL_STANDOFF_M * approach_dir
                )
                grasp_attempt += 1
                break

            for grasp_offset in grasp_retry_offsets:
                grasp_attempt += 1
                # 2-step 구조에서 grasp endpoint는 pre-approach보다 target에
                # 가까워야 한다. 6cm pre에서 7cm offset을 허용하면 직선 진입이
                # 음수가 되어 정확도 보장 목적이 깨진다.
                if grasp_offset >= PRE_APPROACH_OFFSET:
                    continue
                ee_g_try = straw - (
                    grasp_offset + self._ee_to_tcp_offset_m
                ) * approach_dir
                r_grasp = self.plan(pre_joints, ee_g_try.tolist(), q_retry, num_ik_seeds=32)
                if r_grasp is None:
                    continue
                ret_pre   = r_pre_for_variant
                ret_grasp = r_grasp
                used_grasp_offset = grasp_offset
                used_grasp_variant = (quat_frame, axis, quat_deg)
                used_approach_dir = approach_dir
                used_grasp_ee_pos = ee_g_try.copy()
                break
            if ret_pre is not None:
                break

        if (
            ret_pre is not None
            and raw_straw[0] < -0.30
            and used_grasp_offset >= 0.050
        ):
            self.get_logger().warn(
                f"LEFTMOST_DEPTH_LIMITED: deeper 30/35/40/45mm endpoints rejected; "
                f"using {used_grasp_offset*1000:.0f}mm stand-off")
            self.runtime_log.log(
                "leftmost_depth_limited",
                selected_grasp_offset_m=used_grasp_offset,
                attempted_offsets_m=[
                    value for value in LEFTMOST_GRASP_RETRY_OFFSETS
                    if value < used_grasp_offset
                ],
                reason="deeper_endpoints_rejected",
            )

        if ret_pre is None:
            self.get_logger().error(
                f"ABORT: grasp 전체 실패 — {grasp_attempt}개 후보 모두 reject "
                f"(target=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
                f"start_J=[{', '.join(f'{np.rad2deg(v):.0f}' for v in self.current_joints)}]°)")
            self._clear_neighbor_obstacles()
            self._reset_gripper()
            self.pick_complete_pub.publish(Empty())
            return

        if self._measured_tcp_model and self._measured_tcp_plan_only:
            self.get_logger().warn(
                "MEASURED_TCP_PLAN_ONLY: valid pre-approach found and guarded "
                f"{(PRE_APPROACH_OFFSET - used_grasp_offset)*1000:.0f}mm final "
                "MoveLine prepared; "
                "no robot motion dispatched. Set measured_tcp_plan_only:=false only "
                "after reviewing the target and keeping E-stop ready.")
            self.runtime_log.log(
                "measured_tcp_plan_only_hold",
                grasp_offset_m=used_grasp_offset,
                grasp_variant=used_grasp_variant,
                approach_dir=used_approach_dir,
                planned_pre_endpoint_rad=ret_pre[0][-1].tolist(),
                planned_grasp_endpoint_rad=ret_grasp[0][-1].tolist(),
                final_standoff_m=used_grasp_offset,
                guarded_final_move_line_m=PRE_APPROACH_OFFSET - used_grasp_offset,
                pick_complete_published=False,
            )
            self._clear_neighbor_obstacles()
            self.get_logger().warn(
                "MEASURED_TCP_PLAN_ONLY_HOLD: /pick_complete was not published, "
                "so the scan executor must not return home or advance automatically.")
            return

        # pre-approach 실행 후 직선 진입
        if self._measured_tcp_model:
            # 딸기마다 실제 깊이가 다름 → raw detection Y로 진입 거리 개별 계산
            # baseline(180mm)보다 깊은 딸기만 추가 진입; baseline 미만으로 줄이지 않음
            baseline_approach = PRE_APPROACH_OFFSET - MEASURED_TCP_FINAL_STANDOFF_M  # 0.180m
            pre_approach_y_m = WALL_SURFACE_Y_M - PRE_APPROACH_OFFSET  # 0.612m
            adaptive_dist = (detection_raw_y - Y_DETECTION_BIAS_M) - pre_approach_y_m
            final_approach_distance = max(baseline_approach, min(adaptive_dist, 0.260))
        else:
            final_approach_distance = PRE_APPROACH_OFFSET - used_grasp_offset
        if not self.execute_spline(*ret_pre):
            self.get_logger().error("ABORT: pre-approach spline 실패")
            self._clear_neighbor_obstacles()
            self._reset_gripper()
            self.pick_complete_pub.publish(Empty())
            return
        self.get_logger().info(
            f"PRE_APPROACH_REACHED — settling {PRE_APPROACH_SETTLE_SEC:.1f}s "
            f"before {final_approach_distance*1000:.0f}mm straight approach")
        time.sleep(PRE_APPROACH_SETTLE_SEC)

        if final_approach_distance > 0.001:
            if not self.execute_tool_z_line(
                    final_approach_distance,
                    min_distance_m=0.005):
                self.get_logger().error("ABORT: 직선 진입 실패")
                self._clear_neighbor_obstacles()
                self._reset_gripper()
                self.pick_complete_pub.publish(Empty())
                return

        # 실기 확인: 모든 벽면 딸기 줄기는 모델 벽 앞면보다 ~30mm 안쪽에 위치.
        # wall_margin=-30mm이면 available = offset+30mm → 80mm extra 자동 실행.
        # rightmost(x>250mm)는 offsets[-0.03, 0.0]으로 이미 깊게 진입하므로 제외.
        extra_advance_m = 0.0
        if (
            raw_straw[0] <= 0.25
            and self._leftmost_extra_advance_request_m > 0.0
        ):
            available_extra_m = max(
                0.0, used_grasp_offset - self._leftmost_wall_safety_margin_m)
            extra_advance_m = (
                self._leftmost_extra_advance_request_m
                if self._leftmost_allow_wall_model_override
                else min(self._leftmost_extra_advance_request_m, available_extra_m)
            )
            if self._leftmost_allow_wall_model_override:
                modeled_overtravel_m = max(0.0, extra_advance_m - available_extra_m)
                self.get_logger().error(
                    f"LEFTMOST_WALL_MODEL_OVERRIDE: executing "
                    f"{extra_advance_m*1000:.0f}mm extra advance; "
                    f"modeled wall overtravel={modeled_overtravel_m*1000:.0f}mm. "
                    f"Physical clearance and E-stop must be verified.")
                self.runtime_log.log(
                    "leftmost_wall_model_override",
                    requested_m=self._leftmost_extra_advance_request_m,
                    executed_m=extra_advance_m,
                    safe_available_m=available_extra_m,
                    modeled_wall_overtravel_m=modeled_overtravel_m,
                    reason="explicit_ros_parameter_override",
                )
            if extra_advance_m < 0.020:
                self.get_logger().warn(
                    f"LEFTMOST_EXTRA_ADVANCE_BLOCKED: request="
                    f"{self._leftmost_extra_advance_request_m*1000:.0f}mm, "
                    f"safe_available={available_extra_m*1000:.0f}mm, "
                    f"wall_margin={self._leftmost_wall_safety_margin_m*1000:.0f}mm")
                self.runtime_log.log(
                    "leftmost_extra_advance_blocked",
                    requested_m=self._leftmost_extra_advance_request_m,
                    safe_available_m=available_extra_m,
                    wall_safety_margin_m=self._leftmost_wall_safety_margin_m,
                    selected_grasp_offset_m=used_grasp_offset,
                )
                extra_advance_m = 0.0
            else:
                if extra_advance_m < self._leftmost_extra_advance_request_m:
                    self.get_logger().warn(
                        f"LEFTMOST_EXTRA_ADVANCE_CAPPED: request="
                        f"{self._leftmost_extra_advance_request_m*1000:.0f}mm -> "
                        f"execute={extra_advance_m*1000:.0f}mm "
                        f"(wall margin {self._leftmost_wall_safety_margin_m*1000:.0f}mm)")
                self.runtime_log.log(
                    "leftmost_extra_advance",
                    requested_m=self._leftmost_extra_advance_request_m,
                    executed_m=extra_advance_m,
                    wall_safety_margin_m=self._leftmost_wall_safety_margin_m,
                    selected_grasp_offset_m=used_grasp_offset,
                    validation="wall_distance_gate_only_not_curobo_endpoint",
                )
                if not self.execute_tool_z_line(
                    extra_advance_m,
                    motion_label="LEFTMOST_EXTRA_ADVANCE",
                    vel_mm_s=LEFTMOST_EXTRA_ADVANCE_VEL_MM_S,
                    acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
                ):
                    self.get_logger().error(
                        "ABORT: leftmost extra advance failed after dispatch — "
                        "holding current pose")
                    self._clear_neighbor_obstacles()
                    self._hold_pick_sequence("leftmost_extra_advance_failed")
                    return
                used_grasp_ee_pos = (
                    used_grasp_ee_pos + extra_advance_m * used_approach_dir)

        grasp_joints = (
            list(self.current_joints)
            if self.current_joints is not None
            else ret_grasp[0][-1].tolist()
        )
        self.get_logger().info(
            f"GRASP_POSE_REACHED — offset={used_grasp_offset:+.3f}m "
            f"pre={PRE_APPROACH_OFFSET*100:.0f}cm+{final_approach_distance*1000:.0f}mm+{extra_advance_m*1000:.0f}mm "
            f"variant={used_grasp_variant} elevation={np.degrees(np.arcsin(np.clip(used_approach_dir[2], -1.0, 1.0))):+.1f}deg "
            f"(attempt {grasp_attempt}/{n_offsets * n_quats})")
        self.runtime_log.log(
            "grasp_pose_reached",
            grasp_offset_m=used_grasp_offset,
            grasp_variant=used_grasp_variant,
            approach_dir=used_approach_dir,
            extra_advance_m=extra_advance_m,
            current_joints_rad=self.current_joints,
        )

        # 크레인 접근: 벽 앞에서 잎 canopy 높이로 하강 (TOOL+Z 진입 후 줄기 위에서 내려옴)
        if self._measured_tcp_model and CRANE_Z_OFFSET_M > 0:
            if not self.execute_base_z_relative(
                    -CRANE_Z_OFFSET_M, "CRANE_DESCENT", CRANE_DESCENT_VEL_MM_S):
                self.get_logger().error("ABORT: crane descent 실패")
                self._clear_neighbor_obstacles()
                self._reset_gripper()
                self.pick_complete_pub.publish(Empty())
                return

        # 3. 그리퍼 닫기
        self.get_logger().info("3 close gripper")
        self.runtime_log.log("gripper_command", command="close")
        self.call_trigger(self.cli_gripper_close)
        time.sleep(1.5)

        # 3b. VERIFY_GRASP — 실제 그리퍼 위치를 읽어 파지 여부 판정
        grasp_result, present_pos, present_current_raw, grasp_reason = self._verify_grasp()
        self.get_logger().info(
            f"VERIFY_GRASP: {grasp_result} present_pos={present_pos} "
            f"current_raw={present_current_raw} — {grasp_reason}")
        self.runtime_log.log(
            "verify_grasp",
            result_code=grasp_result,
            present_position=present_pos,
            present_current_raw=present_current_raw,
            reason=grasp_reason,
            close_command_pos=700,
            empty_threshold=GRASP_EMPTY_POSITION_THRESHOLD,
            current_contact_threshold_raw=self._grasp_current_contact_threshold_raw,
        )

        # 4. BASE -Z 당기기로 줄기 분리 후 직선 역진 retreat
        self.get_logger().info(
            f"4 detach pull — BASE -Z {DETACH_PULL_DOWN_MM:.0f}mm "
            f"at {DETACH_PULL_VEL_MM_S:.0f}mm/s")
        self._execute_pitch_detach()  # 실패해도 retreat은 항상 실행

        # 실측 TCP 모델은 cuRobo가 pre-approach까지만 계획하므로 최종 MoveLine
        # 전체를 역진한다. Legacy 모델은 기존 검증 baseline대로 extra advance만
        # 역진하고 이후 joint-space 복귀를 사용한다.
        reverse_distance_m = extra_advance_m
        if self._measured_tcp_model:
            reverse_distance_m += final_approach_distance
        reverse_ok = True
        if reverse_distance_m > 0.0:
            reverse_ok = self.execute_tool_z_line(
                -reverse_distance_m,
                motion_label="RETREAT",
                vel_mm_s=RETREAT_VEL_MM_S,
                acc_mm_s2=RETREAT_ACC_MM_S2,
            )
        if not reverse_ok:
            self.get_logger().error(
                "ABORT: straight reverse retreat failed — holding current pose")
            self._clear_neighbor_obstacles()
            self._hold_pick_sequence("straight_reverse_retreat_failed")
            return

        time.sleep(STRAIGHT_RETREAT_SETTLE_SEC)
        retreat_joints = (
            list(self.current_joints)
            if self.current_joints is not None
            else grasp_joints
        )

        # 4b. VERIFY_DETACH
        detach_result = "DETACH_UNVERIFIED"
        self.runtime_log.log(
            "verify_detach",
            result_code=detach_result,
            grasp_result=grasp_result,
            retreat_policy="pitch_detach_then_straight_reverse",
            reason="no sensor; pitch detach executed",
        )

        # Place 게이트 기본값은 fail-closed다. 센서 판독이 불가능한 실험에서만
        # allow_unverified_grasp_place를 명시적으로 켜고 사람 관찰 라벨을 남긴다.
        _allow_place = (
            grasp_result == "GRASP_CONTACT_DETECTED"
            or (
                grasp_result == "GRASP_UNVERIFIED"
                and self._allow_unverified_grasp_place
            )
        )
        if not _allow_place:
            place_block_reason = (
                "GRASP_EMPTY: jaw fully closed, nothing grabbed"
                if grasp_result == "GRASP_EMPTY"
                else "GRASP_UNVERIFIED: enable explicit override only after visual check"
            )
            self.get_logger().warn(
                f"PLACE_GATE_BLOCKED ({grasp_result}): {place_block_reason}")
            self.runtime_log.log(
                "place_gate_blocked",
                grasp_result=grasp_result,
                reason=place_block_reason,
            )

        return_start_joints = retreat_joints
        if self._enable_marker_place and _allow_place:
            place_status, place_joints = self._execute_marker_place_after_retreat(
                retreat_joints)
            if place_status == "success":
                return_start_joints = place_joints
            elif place_status == "skip":
                # tray 없음/stale — place 생략, scan 복귀
                self.get_logger().warn("PLACE_SKIPPED: tray unavailable; returning to scan")
                self.runtime_log.log("place_skipped", reason="tray_unavailable",
                                     grasp_result=grasp_result)
            else:
                # 로봇이 이미 움직인 뒤 실패 or preview hold → latch
                self._clear_neighbor_obstacles()
                self.runtime_log.log(
                    "pick_sequence_stopped",
                    result_code=(
                        "MARKER_PLACE_PREVIEW_HOLD"
                        if place_status == "preview_hold"
                        else "MARKER_PLACE_FAILED"
                    ),
                    place_status=place_status,
                    current_joints_rad=self.current_joints,
                )
                self.get_logger().warn(
                    f"PICK_SEQUENCE_HOLD place_status={place_status}; "
                    "pick_complete not published, automatic scan paused")
                self._hold_pick_sequence(f"marker_place_{place_status}")
                return

        self.get_logger().info("7 return to pick-start scan pose")
        # 직선 retreat 또는 marker place 완료 후 이번 pick이 시작된 scan pose로
        # 복귀한다. scan_executor는 같은 SW 셀의 다음 target을 이어서 전달한다.
        pick_start_joints_deg = np.rad2deg(pick_start_joints).tolist()
        pick_start_joints_deg = self._nearest_equivalent_joints(pick_start_joints_deg)
        ok, _ = self.plan_to_fixed_joints_pose(
            return_start_joints, pick_start_joints_deg, "pick-start scan pose after pick/place",
            skip_swing_check=True)
        if not ok:
            self.get_logger().warn(
                "pick-start scan pose after pick/place failed; holding current pose")
            self._clear_neighbor_obstacles()
            self.runtime_log.log(
                "pick_sequence_stopped",
                result_code="RETURN_TO_SCAN_FAILED",
                current_joints_rad=self.current_joints,
            )
            self._hold_pick_sequence("return_to_scan_failed")
            return

        self._clear_neighbor_obstacles()
        self._reset_gripper()  # 다음 파지를 위해 approach 위치(600)로 복귀
        self.pick_complete_pub.publish(Empty())
        sequence_result_code = (
            "DETACH_SUCCESS_UNVERIFIED"
            if grasp_result == "GRASP_CONTACT_DETECTED"
            else grasp_result   # GRASP_EMPTY or GRASP_UNVERIFIED
        )
        self.runtime_log.log(
            "pick_sequence_complete",
            result_code=sequence_result_code,
            grasp_result=grasp_result,
            detach_result=detach_result,
            return_pose="pick_start_scan_pose",
            marker_place_enabled=self._enable_marker_place,
            marker_place_release_executed=(
                self._enable_marker_place and self._execute_marker_place_release),
            current_joints_rad=self.current_joints,
        )
        self.get_logger().info(f"=== PICK COMPLETE ({sequence_result_code}) ===")


def main():
    rclpy.init()
    node = CuroboPlanner()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
