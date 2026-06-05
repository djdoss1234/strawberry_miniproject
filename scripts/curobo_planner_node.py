#!/usr/bin/env python3
"""
cuRobo Motion Planner Node for Doosan E0509

Pick sequence:
  open → approach(CuRobo 15cm) → grasp(CuRobo) → close
       → [check] → retreat(CuRobo) → bin → home → pick_complete
"""

import os
import time
import torch
import numpy as np
import json
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, String, Int32, Empty
from std_srvs.srv import Trigger
from dsr_msgs2.srv import MoveSplineJoint, MoveJoint

from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState as CuroboJointState, RobotConfig
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.geom.types import WorldConfig, Cuboid, Sphere


# ── 딸기 접근 파라미터 ────────────────────────────────────────────────────────
APPROACH_OFFSET  = 0.18    # legacy staged approach distance (disabled during direct-grasp harvest test)
STAGING_EXTRA    = 0.12    # legacy staging distance (disabled during direct-grasp harvest test)
CLOSE_CONFIRM_OFFSET = 0.10 # far-view target lock 후 grasp 10cm 전에서 close-view geometry 확인
GRASP_OFFSET     = +0.050   # TCP를 KP0보다 5cm 앞에 세움 — 15.8cm extension이 벽(672mm)에서 26mm 여유
GRASP_RETRY_OFFSETS = [0.050, 0.065, 0.080, 0.100]  # deep grasp first; back off if IK cannot reach
RETREAT_OFFSET   = 0.36    # demo: 딸기/벽에서 더 빠져 place 이동 중 스침을 줄임
RETREAT_UP_M     = 0.05    # retreat 시 위쪽 5cm 추가 — 이웃 딸기 스침 방지
NEIGHBOR_SPHERE_RADIUS_M = 0.030  # 이웃 딸기 장애물 sphere 반지름 (30mm)
PRE_BIN_CLEAR_OFFSET = 0.42 # place 이동 전 벽에서 충분히 빠지는 clear 지점
PRE_BIN_CLEAR_RETRY_OFFSETS = [0.42, 0.36, 0.30]
GRASP_Z_BIAS     = +0.025  # grasp stem 2.5cm above each berry's KP0
USE_STAGING      = False   # harvest test: current pose → grasp in one cuRobo motion
USE_APPROACH     = False   # harvest test: skip staging/approach segmentation
USE_CLOSE_CONFIRM = False  # close view is unstable today; SW first pass uses far-view lock -> direct grasp
CLOSE_CONFIRM_DWELL_SEC = 1.2
CLOSE_CONFIRM_TIMEOUT_SEC = 2.5
CLOSE_CONFIRM_MAX_TARGET_DRIFT_M = 0.08
USE_PRE_BIN_CLEAR = False  # demo: retreat 후 place로 바로 이동해 불필요한 우회/딸기 접촉을 줄임
ENABLE_PLACE_SEQUENCE = False  # table collision risk: hold after retreat; do not move to egg tray yet
USE_CUROBO_FIXED_POSES = True  # True: home/place 고정 joint 자세도 cuRobo joint-space로 계획
USE_MOVEJ_FOR_DEMO_PLACE = True # demo: 계란판 위 짧은 release/home만 MoveJoint, 큰 이동은 cuRobo 유지
ALLOW_MOVEJ_FALLBACK = False   # True: cuRobo 실패 시 MoveJoint로 후퇴. 100% cuRobo 검증 중에는 False 권장
USE_CUROBO_SELF_COLLISION = False  # 현재 coarse sphere 모델은 정상 자세도 self-collision으로 오검출함
USE_PLACED_STRAWBERRY_OBSTACLES = False  # demo: 놓인 딸기 obstacle은 영상 성공 후 다시 켜서 검증
MAX_SPLINE_POINTS = 12         # Doosan spline point가 너무 많으면 실기에서 뚝뚝 끊겨 보임
SPLINE_TIME_SCALE = 1.125      # 1/3 속도 (cuRobo plan_time × 1.125)
SPLINE_MIN_TIME = 0.75
USE_TRAY_ENTRY_WAYPOINT = False # demo: 각 slot above로 바로 이동
USE_LEFT_SAFE_TRANSFER = True   # 맨 왼쪽 딸기는 place 전 안전 자세를 거쳐 주변 딸기 스침을 줄임
LEFT_SAFE_TRANSFER_X_MAX = -0.10
DEBUG_START_COLLISION = True   # INVALID_START_STATE_WORLD_COLLISION 원인 obstacle 분리 로그

# 실제 실험 셀에서는 같은 ee pose라도 과도하게 뒤집힌 joint branch가 벽 충돌을 만들 수 있다.
# cuRobo 성공 결과라도 아래 운용 범위를 벗어나면 실행하지 않는다.
OPERATIONAL_JOINT_LIMITS_DEG = [
    (-225.0, 225.0),   # J1: 오른쪽 딸기 branch는 200도 근처까지 필요
    (-95.0, 95.0),
    (-155.0, 155.0),
    (-270.0, 270.0),   # J4: SW scan pose = 262.2° → ±175°로 제한하면 정규화 후 -97.8°로 변환 → Doosan 360° 스핀 발생
    (-130.0, 130.0),
    (-225.0, 225.0),   # J6: wrist wrap은 180도 근처까지 허용
]
# Only wrist joints are normalized.  Do not rewrite J1: a base branch such as
# 262 deg is not an acceptable equivalent during harvest because converting it
# to -98 deg makes the robot swing around the opposite side of the board.
WRAP_EQUIVALENT_JOINT_IDX = {3, 5}
MAX_HARVEST_JOINT_DELTA_DEG = [
    75.0,   # J1: stay on the current cell side
    90.0,
    120.0,
    150.0,
    130.0,
    180.0,
]

GRIPPER_LEN      = 0.160   # ee_link → TCP 거리 (m)
WALL_SURFACE_Y_M = 0.672   # whiteboard 전면 Y (environment.yaml center=0.682 - half-thickness 0.01)
                            # Berry는 이 값보다 멀(뒤) 수 없음 — FK 미보정 오차 클램핑에 사용
WALL_UNIT        = np.array([-0.035, 0.996, -0.084])   # 티치펜던트 실측 (2026-05-18)
# ee_link [w,x,y,z].  Updated from the 2026-06-01 gripper-centered scan poses
# (rx≈88°, ry≈86°, rz≈-89°).  The previous 2026-05-18 wall-facing quaternion
# no longer matched the retaught gripper-centered harvest posture and produced
# IK_FAIL for SW stem targets.
WALL_QUAT_WXYZ   = [0.548415, -0.439294, 0.424628, 0.570923]  # 2026-05-18 실측; tool-Z ≈ WALL_UNIT (+Y벽방향)
GRASP_QUAT_RETRY_DEG = [0.0, -8.0, 8.0, -15.0, 15.0]  # small wrist pitch search for close SW reachability
CARTESIAN_PLAN_MAX_ATTEMPTS = 2  # unreachable grasp target should not stall for 10s x retries
CARTESIAN_PLAN_TIMEOUT_SEC = 1.2
DIRECT_GRASP_TARGET_X_RANGE_M = (-0.45, 0.45)  # allow left SW berries; reject only obvious off-board picks

# ── 고정 자세 ─────────────────────────────────────────────────────────────────
HOME_JOINTS_DEG  = [88.0, -80.0, 130.0, 0.0, 20.0, -90.0]
LEFT_SAFE_TRANSFER_JOINTS_DEG = HOME_JOINTS_DEG
BIN_JOINTS_DEG   = [0.0, 65.0, 25.0, 0.0, 90.0, 0.0]
PLACE_SLOTS = [
    {
        "name": "slot0",
        "above": {"joints": BIN_JOINTS_DEG},
        "release": {"joints": BIN_JOINTS_DEG},
    },
]
def resolve_place_slots_yaml():
    """개발 중에는 install 복사본보다 src/config를 단일 진실로 사용한다."""
    candidates = [
        os.path.expanduser("~/doosan_ws/src/e0509_gripper_description/config/place_slots.yaml"),
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "place_slots.yaml",
        ),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]


PLACE_SLOTS_YAML = resolve_place_slots_yaml()


def resolve_environment_yaml():
    """RViz/MoveIt visualizer와 같은 environment.yaml을 cuRobo world에도 사용한다."""
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

# ── 파지 성공 판별 ────────────────────────────────────────────────────────────
USE_GRASP_CHECK  = False  # /gripper/stroke는 현재 명령값이라 실제 파지 판정에 쓰지 않음
GRASP_STROKE_MIN = 10   # stroke 이하면 파지 실패 (완전 닫힘)

# ── 그리퍼 soft close ─────────────────────────────────────────────────────────
USE_SOFT_CLOSE    = False  # False: trigger full close in one shot
OPEN_GRIPPER_ON_PICK_START = True
GRIPPER_PRE_CLOSE_POS = 300     # 접촉 전 1차 닫힘
GRIPPER_CONTACT_POS   = 420     # 스퀴지 딸기 표면 접촉 위치
GRIPPER_HARVEST_POS   = 580     # demo: 너무 타이트한 파지를 완화
GRIPPER_CLOSE_STEPS = [
    ("pre", GRIPPER_PRE_CLOSE_POS),
    ("contact", GRIPPER_CONTACT_POS),
    ("harvest", GRIPPER_HARVEST_POS),
]
GRIPPER_STEP_DELAY = 0.7               # demo: gripper_service_node가 명령 처리할 시간


def _float_list(values):
    if values is None:
        return None
    return [float(v) for v in values]


def _load_slot_target(data):
    """Load one slot target.

    `joints_deg` is the executable target. `posx` is kept only as measured TCP
    metadata, mainly for adding placed-strawberry obstacles in the real tray.
    We intentionally do not synthesize a Cartesian pose from joints here: doing
    so can create a believable but wrong world model.
    """
    data = data or {}
    target = {
        "joints": _float_list(data.get("joints_deg")),
        "posx": _float_list(data.get("posx")),
    }
    pose = data.get("pose")
    if pose is not None:
        target["pose"] = {
            "pos": _float_list(pose.get("pos")),
            "quat": _float_list(pose.get("quat")),
        }
    else:
        target["pose"] = None
    return target


def load_place_slots():
    if not os.path.exists(PLACE_SLOTS_YAML):
        return PLACE_SLOTS
    with open(PLACE_SLOTS_YAML, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    slots = []
    for item in data.get("slots", []):
        if not item.get("enabled", True):
            continue
        if item.get("occupied", False):
            continue
        name = item.get("name", f"slot{len(slots)}")
        above = _load_slot_target(item.get("above"))
        release = _load_slot_target(item.get("release"))
        if above["joints"] is None and above["pose"] is None:
            continue
        if release["joints"] is None and release["pose"] is None:
            continue
        slots.append({
            "name": name,
            "above": above,
            "release": release,
        })
    return slots or PLACE_SLOTS


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


def load_environment_cuboids():
    """environment.yaml의 enabled cuboid를 cuRobo WorldConfig용 Cuboid로 변환한다."""
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
            print(f"[WARN] invalid environment object skipped: {obj.get('name', '?')} ({e})")
    if not cuboids:
        cuboids.append(Cuboid(name="table", pose=[0.0, 0.0, -0.02, 1, 0, 0, 0], dims=[1.2, 1.2, 0.04]))
    return cuboids


class CuroboPlanner(Node):

    JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    JOINT_LIMITS = [
        (-6.273185, 6.273185),
        (-1.648063, 1.648063),
        (-2.6953,   2.6953  ),  # J3: ±155°
        (-6.273185, 6.273185),
        (-2.346194, 2.346194),  # J5: ±135°
        (-6.273185, 6.273185),
    ]

    def __init__(self):
        super().__init__("curobo_planner_node")

        self.service_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.current_joints = None
        self.gripper_stroke = None
        self._pick_busy = False
        self.place_slot_idx = 0
        self.place_slots = load_place_slots()
        self.static_cuboids = load_environment_cuboids()
        self.dynamic_cuboids = []
        self.placed_cuboids = []
        self.neighbor_spheres: list = []   # 현재 씬 이웃 딸기 장애물 (pick마다 갱신)
        self._scene_positions: list = []   # /strawberry/detection/scene_positions 수신값
        self._latest_pick_pose: PoseStamped | None = None
        self._latest_pick_pose_time = 0.0
        self._latest_detection_pose: PoseStamped | None = None
        self._latest_detection_pose_time = 0.0

        # ── cuRobo 초기화 ─────────────────────────────────────────────────────
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
        with open(os.path.join(config_dir, "e0509_gripper.yml"), "r", encoding="utf-8") as f:
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

        # ── ROS2 인터페이스 ───────────────────────────────────────────────────
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
            PoseStamped, "/strawberry/detection/pick_pose", self._detection_pick_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            String, "/dsr01/curobo/obstacles", self.obstacles_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            Int32, "/dsr01/gripper/stroke", self._stroke_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            Float64MultiArray, "/strawberry/detection/scene_positions", self._scene_cb, 10,
            callback_group=self.service_cb_group)

        self.pick_complete_pub = self.create_publisher(Empty, "/dsr01/curobo/pick_complete", 10)
        self.vla_request_pub = self.create_publisher(PoseStamped, "/strawberry/vla/request", 10)
        self.gripper_pos_pub = self.create_publisher(Int32, "/dsr01/gripper/position_cmd", 10)

        self.cli_spline = self.create_client(
            MoveSplineJoint, "/dsr01/motion/move_spline_joint",
            callback_group=self.service_cb_group)
        self.cli_movej = self.create_client(
            MoveJoint, "/dsr01/motion/move_joint",
            callback_group=self.service_cb_group)
        self.cli_gripper_open = self.create_client(
            Trigger, "/dsr01/gripper/open", callback_group=self.service_cb_group)
        self.cli_gripper_close = self.create_client(
            Trigger, "/dsr01/gripper/close", callback_group=self.service_cb_group)

        self.get_logger().info("cuRobo Planner Ready!")
        self.get_logger().info(
            f"  GRASP_STROKE_MIN={GRASP_STROKE_MIN}  "
            f"PLACE_SLOTS={len(self.place_slots)}")
        self.get_logger().info(
            f"  ENV_CUBOIDS={len(self.static_cuboids)}  "
            f"CUROBO_FIXED={USE_CUROBO_FIXED_POSES}  MOVEJ_FALLBACK={ALLOW_MOVEJ_FALLBACK}  "
            f"SELF_COLLISION={USE_CUROBO_SELF_COLLISION}")
        if os.path.exists(PLACE_SLOTS_YAML):
            self.get_logger().info(f"  place slots loaded: {PLACE_SLOTS_YAML}")
        if os.path.exists(ENVIRONMENT_YAML):
            self.get_logger().info(f"  environment loaded: {ENVIRONMENT_YAML}")

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def joint_state_cb(self, msg: JointState):
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        joints = [jmap.get(n) for n in self.JOINT_NAMES]
        if None not in joints:
            self.current_joints = joints

    def _stroke_cb(self, msg: Int32):
        self.gripper_stroke = msg.data

    def _detection_pick_cb(self, msg: PoseStamped):
        """Latest live fusion output.

        `/dsr01/curobo/pick_pose` is a trigger relayed by scan_executor and is
        not continuously published during the pick.  Close-confirm needs the
        live detector topic so it can verify geometry after the robot moves
        closer without starting a new pick recursively.
        """
        self._latest_detection_pose = msg
        self._latest_detection_pose_time = time.monotonic()

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

    # ── 핵심 메서드 ───────────────────────────────────────────────────────────

    def update_curobo_world(self, reason="manual"):
        cuboids = self.static_cuboids + self.dynamic_cuboids + self.placed_cuboids
        self.motion_gen.update_world(WorldConfig(cuboid=cuboids, sphere=self.neighbor_spheres))
        self.get_logger().info(
            f"World updated ({reason}): static={len(self.static_cuboids)} "
            f"dynamic={len(self.dynamic_cuboids)} placed={len(self.placed_cuboids)} "
            f"neighbor_spheres={len(self.neighbor_spheres)}")

    def _scene_cb(self, msg: Float64MultiArray) -> None:
        """Receive all detected berry positions as flat [x,y,z, x,y,z, ...] array."""
        data = msg.data
        positions = []
        for i in range(0, len(data) - 2, 3):
            positions.append(np.array([data[i], data[i + 1], data[i + 2]]))
        self._scene_positions = positions

    def _register_neighbor_obstacles(self, target_pos: np.ndarray) -> None:
        """Register all scene positions except the target as sphere obstacles."""
        spheres = []
        for i, pos in enumerate(self._scene_positions):
            if np.linalg.norm(pos - target_pos) < 0.035:
                continue  # skip target itself
            spheres.append(Sphere(
                name=f"neighbor_{i}",
                pose=[float(pos[0]), float(pos[1]), float(pos[2]), 1.0, 0.0, 0.0, 0.0],
                radius=NEIGHBOR_SPHERE_RADIUS_M,
            ))
        self.neighbor_spheres = spheres
        self.update_curobo_world("neighbor obstacles registered")
        self.get_logger().info(f"Registered {len(spheres)} neighbor sphere obstacle(s)")

    def _clear_neighbor_obstacles(self) -> None:
        """Remove neighbor spheres from cuRobo world after pick."""
        if self.neighbor_spheres:
            self.neighbor_spheres = []
            self.update_curobo_world("neighbor obstacles cleared")

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
            # check_start_state toggles internal constraints while checking. Make
            # the diagnostic side-effect free before the next real plan.
            try:
                self.motion_gen.rollout_fn.primitive_collision_constraint.enable_cost()
                self.motion_gen.rollout_fn.robot_self_collision_constraint.enable_cost()
            except Exception:
                pass

    def diagnose_start_world_collision(self, joints, label):
        if not DEBUG_START_COLLISION:
            return

        full_world = self.static_cuboids + self.dynamic_cuboids + self.placed_cuboids
        far_dummy = Cuboid(
            name="debug_far_dummy",
            pose=[10.0, 10.0, 10.0, 1.0, 0.0, 0.0, 0.0],
            dims=[0.01, 0.01, 0.01],
        )
        tests = [("empty_world", [far_dummy])]
        tests += [(f"static:{c.name}", [c]) for c in self.static_cuboids]
        tests += [(f"dynamic:{c.name}", [c]) for c in self.dynamic_cuboids]
        tests += [(f"placed:{c.name}", [c]) for c in self.placed_cuboids]

        bad = []
        try:
            for name, cuboids in tests:
                feasible, status = self._check_state_feasible_with_world(joints, cuboids)
                self.get_logger().warn(
                    f"{label} collision diag {name}: "
                    f"{'OK' if feasible else 'COLLISION'} "
                    f"status={status}")
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

    def add_placed_strawberry_obstacle(self, slot_name, place_slot):
        if not USE_PLACED_STRAWBERRY_OBSTACLES:
            self.get_logger().info(f"Placed obstacle skipped in demo mode: {slot_name}")
            return
        release_posx = (place_slot.get("release") or {}).get("posx")
        if release_posx is None:
            self.get_logger().warn(f"Placed obstacle skipped: {slot_name} has no release posx")
            return
        pos = [release_posx[0] / 1000.0, release_posx[1] / 1000.0, release_posx[2] / 1000.0]
        pos[2] += 0.035
        self.placed_cuboids.append(Cuboid(
            name=f"placed_{slot_name}",
            pose=[pos[0], pos[1], pos[2], 1, 0, 0, 0],
            dims=[0.06, 0.06, 0.07],
        ))
        self.update_curobo_world(f"placed {slot_name}")

    def set_held_strawberry_collision(self, enabled):
        if enabled:
            spheres = torch.tensor(
                [
                    [0.0, 0.0, 0.00, 0.026],
                    [0.0, 0.0, 0.025, 0.020],
                    [0.0, 0.0, 0.00, -100.0],
                    [0.0, 0.0, 0.00, -100.0],
                ],
                device="cuda:0",
                dtype=torch.float32,
            )
            self.motion_gen.attach_spheres_to_robot(
                sphere_tensor=spheres,
                link_name="attached_object",
            )
        else:
            self.motion_gen.detach_object_from_robot()

    def _clamp_joints(self, joints):
        return [float(np.clip(j, lo, hi)) for j, (lo, hi) in zip(joints, self.JOINT_LIMITS)]

    def grasp_candidates_for_target(self, straw):
        """Demo heuristic: far-right berries waste seconds on infeasible deep IK."""
        if straw[0] > 0.25:
            return [-0.03, 0.0]
        return GRASP_RETRY_OFFSETS

    def trajectory_in_operational_limits(self, traj_rad, label):
        traj_deg = np.rad2deg(traj_rad)
        for joint_idx, (lo, hi) in enumerate(OPERATIONAL_JOINT_LIMITS_DEG):
            vals = traj_deg[:, joint_idx]
            min_v = float(np.min(vals))
            max_v = float(np.max(vals))
            if min_v < lo or max_v > hi:
                self.get_logger().warn(
                    f"{label} rejected: J{joint_idx + 1} operational range "
                    f"{min_v:.1f}~{max_v:.1f}° outside {lo:.1f}~{hi:.1f}°")
                return False
        return True

    def trajectory_has_reasonable_swing(self, traj_rad, start_joints, label):
        traj_deg = np.rad2deg(traj_rad)
        start_deg = np.rad2deg(start_joints)
        for joint_idx, max_delta in enumerate(MAX_HARVEST_JOINT_DELTA_DEG):
            vals = traj_deg[:, joint_idx]
            if joint_idx in WRAP_EQUIVALENT_JOINT_IDX:
                delta_vals = np.abs(((vals - start_deg[joint_idx] + 180.0) % 360.0) - 180.0)
            else:
                delta_vals = np.abs(vals - start_deg[joint_idx])
            delta = float(np.max(delta_vals))
            if delta > max_delta:
                end_deg = traj_deg[-1, joint_idx]
                self.get_logger().warn(
                    f"{label} rejected: J{joint_idx + 1} swing {delta:.1f}° > {max_delta:.1f}° "
                    f"(start={start_deg[joint_idx]:.1f}° → end={end_deg:.1f}°)")
                return False
        return True

    def normalize_trajectory_equivalents(self, traj_rad, label):
        """Rewrite wrap-capable joints to equivalent angles inside op limits.

        cuRobo may return a wrist branch such as J4=262 deg.  Physically this is
        the same pose as -98 deg, but the operational-limit filter sees 262 deg
        as invalid.  Normalize wrist joints before checking limits and before
        sending the path to Doosan.  J1 is intentionally not normalized because
        base wrap changes the side from which the robot approaches the board.
        """
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
                    f"J{joint_idx + 1} {float(np.min(original)):.1f}~{float(np.max(original)):.1f}"
                    f" -> {float(np.min(traj_deg[:, joint_idx])):.1f}~{float(np.max(traj_deg[:, joint_idx])):.1f}"
                )

        if rewritten:
            self.get_logger().info(
                f"{label} joint equivalent rewrite: " + "; ".join(rewritten)
            )
        return np.deg2rad(traj_deg)

    def plan(self, start_joints, target_pos, target_quat_wxyz, num_ik_seeds=32):
        """CuRobo plan_single. 성공 시 (traj ndarray, motion_time_sec) 반환, 실패 시 None."""
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
            start_state,
            target_pose,
            MotionGenPlanConfig(
                num_ik_seeds=num_ik_seeds,
                max_attempts=CARTESIAN_PLAN_MAX_ATTEMPTS,
                timeout=CARTESIAN_PLAN_TIMEOUT_SEC,
                enable_graph_attempt=None,
            ),
        )
        dt = (time.time() - t0) * 1000

        if result.success.item():
            traj = result.get_interpolated_plan().position.cpu().numpy()
            traj = self.normalize_trajectory_equivalents(traj, "Cartesian plan")
            if not self.trajectory_in_operational_limits(traj, "Cartesian plan"):
                return None
            if not self.trajectory_has_reasonable_swing(traj, start_joints, "Cartesian plan"):
                return None
            motion_time = float(result.motion_time.item())
            end_deg = [f"{np.rad2deg(v):.1f}" for v in traj[-1]]
            self.get_logger().info(
                f"Plan OK {dt:.0f}ms {traj.shape[0]}pts {motion_time:.2f}s | "
                f"goal={[f'{v*1000:.0f}' for v in target_pos]}mm | "
                f"end_J=[{', '.join(end_deg)}]°")
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
            return None

    def plan_js(self, start_joints, target_joints_rad, label):
        """Joint 목표로 cuRobo joint-space planning. 고정 자세 이동은 IK를 피하기 위해 이 경로를 쓴다."""
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
                return None
            if not self.trajectory_has_reasonable_swing(traj, start_joints, label):
                return None
            motion_time = float(result.motion_time.item())
            self.get_logger().info(
                f"{label} JS Plan OK {dt:.0f}ms {traj.shape[0]}pts {motion_time:.2f}s | "
                f"goal={[f'{v:.1f}' for v in np.rad2deg(target_joints_rad)]}°")
            return traj, motion_time

        status = getattr(result, "status", "?")
        self.get_logger().error(
            f"{label} JS Plan FAIL {dt:.0f}ms | status={status} | "
            f"goal={[f'{v:.1f}' for v in np.rad2deg(target_joints_rad)]}°")
        if "INVALID_START_STATE_WORLD_COLLISION" in str(status) or "GRAPH_FAIL" in str(status):
            self.diagnose_js_endpoint_collision(start_joints, target_joints_rad, label)
        return None

    def execute_spline(self, traj_rad, motion_time: float) -> bool:
        """CuRobo trajectory(rad)를 MoveSplineJoint로 실행."""
        if not self.cli_spline.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveSplineJoint not available")
            return False

        traj_deg = np.rad2deg(traj_rad)
        n = traj_deg.shape[0]
        if n > MAX_SPLINE_POINTS:
            idx = np.linspace(0, n - 1, MAX_SPLINE_POINTS, dtype=int)
            traj_deg = traj_deg[idx]
            n = MAX_SPLINE_POINTS

        req = MoveSplineJoint.Request()
        req.pos_cnt = n
        for row in traj_deg:
            pt = Float64MultiArray()
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
        future = self.cli_spline.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)

        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error("Spline failed/timeout")
        return ok

    def movej(self, joints_deg, vel=40.0, acc=40.0) -> bool:
        """MoveJoint으로 고정 자세 이동 (bin, home 등)."""
        if not self.cli_movej.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveJoint not available")
            return False
        req = MoveJoint.Request()
        req.pos = joints_deg
        req.vel = vel
        req.acc = acc
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        future = self.cli_movej.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error("MoveJoint failed")
        return ok

    def plan_to_fixed_joints_pose(self, start_joints, target_joints_deg, label, fallback_movej=ALLOW_MOVEJ_FALLBACK):
        """고정 joint 자세는 Cartesian IK가 아니라 cuRobo joint-space planner로 이동한다."""
        if USE_MOVEJ_FOR_DEMO_PLACE and (
            " release" in label
            or "above retreat" in label
            or label == "home"
        ):
            if label == "home":
                ok = self.movej(target_joints_deg, vel=60.0, acc=80.0)
            else:
                ok = self.movej(target_joints_deg, vel=25.0, acc=35.0)
            return ok, np.deg2rad(target_joints_deg).tolist()

        if not USE_CUROBO_FIXED_POSES:
            ok = self.movej(target_joints_deg, vel=20.0, acc=20.0)
            return ok, np.deg2rad(target_joints_deg).tolist()

        target_joints_rad = np.deg2rad(target_joints_deg).tolist()
        ret = self.plan_js(start_joints, target_joints_rad, label)
        if ret is not None and self.execute_spline(*ret):
            return True, ret[0][-1].tolist()

        self.get_logger().warn(f"{label} CuRobo joint-space failed")
        if fallback_movej:
            self.get_logger().warn(f"{label} fallback MoveJoint")
            ok = self.movej(target_joints_deg, vel=20.0, acc=20.0)
            return ok, np.deg2rad(target_joints_deg).tolist()
        return False, start_joints

    def plan_to_pose_target(self, start_joints, target_pose, label):
        """place slot처럼 Cartesian pose로 정의된 목표를 cuRobo로 이동한다."""
        pos = target_pose["pos"]
        quat = target_pose["quat"]
        self.get_logger().info(
            f"{label} CuRobo pose goal={[f'{v*1000:.0f}' for v in pos]}mm")
        ret = self.plan(start_joints, pos, quat)
        if ret is not None and self.execute_spline(*ret):
            return True, ret[0][-1].tolist()
        self.get_logger().error(f"{label} CuRobo failed")
        return False, start_joints

    def move_to_place_target(self, start_joints, place_slot, key, label):
        target = place_slot.get(key) or {}
        if target.get("joints") is not None:
            return self.plan_to_fixed_joints_pose(start_joints, target["joints"], label)
        if target.get("pose") is not None:
            return self.plan_to_pose_target(start_joints, target["pose"], label)
        self.get_logger().error(f"{label}: slot has no {key} target")
        return False, start_joints

    def current_place_slot(self):
        slot_count = len(self.place_slots)
        if slot_count == 0 or self.place_slot_idx >= slot_count:
            return None, None
        idx = min(self.place_slot_idx, slot_count - 1)
        return idx, self.place_slots[idx]

    def tray_entry_slot(self):
        if not self.place_slots:
            return None
        return self.place_slots[0]

    def advance_place_slot(self):
        if self.place_slot_idx < len(self.place_slots) - 1:
            self.place_slot_idx += 1
        else:
            self.place_slot_idx = len(self.place_slots)
            self.get_logger().warn("Place slots exhausted — 다음 pick은 시작하지 않음")

    def call_trigger(self, client):
        if not client.wait_for_service(timeout_sec=3.0):
            return
        future = client.call_async(Trigger.Request())
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 10.0:
            time.sleep(0.1)

    def soft_close_gripper(self):
        """Position command로 단계적으로 닫아 실제 딸기 압상을 줄인다."""
        if not USE_SOFT_CLOSE:
            self.call_trigger(self.cli_gripper_close)
            time.sleep(1.5)
            return

        if self.gripper_pos_pub.get_subscription_count() == 0:
            self.get_logger().warn("No /dsr01/gripper/position_cmd subscriber — Trigger close fallback")
            self.call_trigger(self.cli_gripper_close)
            time.sleep(1.5)
            return

        for label, pos in GRIPPER_CLOSE_STEPS:
            msg = Int32()
            msg.data = int(pos)
            self.gripper_pos_pub.publish(msg)
            self.get_logger().info(f"  soft close {label}: position_cmd={pos}")
            time.sleep(GRIPPER_STEP_DELAY)

    def _check_grasp(self) -> bool:
        """그리퍼 stroke로 파지 성공 여부 판별."""
        if not USE_GRASP_CHECK:
            if self.gripper_stroke is None:
                self.get_logger().info("Grasp check skipped: stroke 데이터 없음")
            else:
                self.get_logger().info(
                    f"Grasp check skipped: stroke={self.gripper_stroke} (명령값 기반)")
            return True
        if self.gripper_stroke is None:
            self.get_logger().warn("Stroke 데이터 없음 — 파지 성공으로 가정")
            return True
        ok = self.gripper_stroke > GRASP_STROKE_MIN
        self.get_logger().info(
            f"Grasp check: stroke={self.gripper_stroke} → {'OK' if ok else 'FAIL (fully closed)'}")
        return ok

    def _wait_for_close_confirmation(self, locked_straw: np.ndarray, since_time: float) -> bool:
        """Confirm that the locked target is still geometrically visible close-up.

        The maturity class is intentionally not re-decided here.  The far scan
        chooses ripe/unripe/sick; close view only checks that the same target
        still has a valid fused pick pose near the locked grasp point.
        """
        deadline = time.monotonic() + CLOSE_CONFIRM_TIMEOUT_SEC
        best_dist = None
        while time.monotonic() < deadline:
            latest = self._latest_detection_pose
            latest_t = self._latest_detection_pose_time
            if latest is not None and latest_t >= since_time:
                p = latest.pose.position
                latest_pos = np.array([p.x, p.y, max(p.z, 0.05)], dtype=float)
                dist = float(np.linalg.norm(latest_pos - locked_straw))
                best_dist = dist if best_dist is None else min(best_dist, dist)
                if dist <= CLOSE_CONFIRM_MAX_TARGET_DRIFT_M:
                    self.get_logger().info(
                        f"CLOSE_CONFIRM_OK target drift={dist*1000:.1f}mm")
                    return True
            time.sleep(0.05)

        if best_dist is None:
            self.get_logger().warn(
                "CLOSE_CONFIRM_FAIL no fresh live detection pick_pose after close-view dwell")
        else:
            self.get_logger().warn(
                f"CLOSE_CONFIRM_FAIL target drift best={best_dist*1000:.1f}mm "
                f"> {CLOSE_CONFIRM_MAX_TARGET_DRIFT_M*1000:.0f}mm")
        return False

    # ── Pick 시퀀스 ───────────────────────────────────────────────────────────

    def pick_pose_cb(self, msg: PoseStamped):
        """open → grasp → close → [check] → retreat.

        Egg-tray place is intentionally disabled while the table collision risk
        is unresolved.  HOME/place recovery will be owned by the later VLA
        retry policy instead of happening on every pick.
        """
        if self.current_joints is None:
            self.get_logger().warn("No joint state yet")
            return
        self._latest_pick_pose = msg
        self._latest_pick_pose_time = time.monotonic()
        if self._pick_busy:
            self.get_logger().warn("Pick already in progress — ignored")
            return
        self._pick_busy = True
        try:
            self._pick(msg)
        finally:
            self._pick_busy = False

    def _pick(self, msg: PoseStamped):
        if ENABLE_PLACE_SEQUENCE:
            next_slot_idx, next_place_slot = self.current_place_slot()
            if next_place_slot is None:
                self.get_logger().error("ABORT: 사용 가능한 place slot 없음 — 계란판 slot 티칭/초기화 필요")
                self.pick_complete_pub.publish(Empty())
                return

        p = msg.pose.position
        raw_y = float(p.y)
        if raw_y > WALL_SURFACE_Y_M:
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
                "ABORT: pick target outside direct-grasp first-pass window "
                f"x={raw_straw[0]*1000:.0f}mm allowed={x_min*1000:.0f}..{x_max*1000:.0f}mm")
            self.pick_complete_pub.publish(Empty())
            return

        # CuRobo ee_link 목표 위치
        ee_s = straw - (APPROACH_OFFSET + STAGING_EXTRA + GRIPPER_LEN) * WALL_UNIT
        ee_a = straw - (APPROACH_OFFSET + GRIPPER_LEN) * WALL_UNIT
        ee_c = straw - (CLOSE_CONFIRM_OFFSET + GRIPPER_LEN) * WALL_UNIT
        grasp_retry_offsets = self.grasp_candidates_for_target(straw)
        ee_g_candidates = [
            (offset, straw - (offset + GRIPPER_LEN) * WALL_UNIT)
            for offset in grasp_retry_offsets
        ]
        # Retreat: pull back + slight upward to avoid sweeping through neighbors
        ee_r = (straw - (RETREAT_OFFSET + GRIPPER_LEN) * WALL_UNIT
                + np.array([0.0, 0.0, RETREAT_UP_M]))
        ee_clear_candidates = [
            (offset, straw - (offset + GRIPPER_LEN) * WALL_UNIT)
            for offset in PRE_BIN_CLEAR_RETRY_OFFSETS
        ]

        self.get_logger().info(
            f"=== PICK 딸기 raw=({raw_straw[0]*1000:.0f},{raw_straw[1]*1000:.0f},{raw_straw[2]*1000:.0f})mm "
            f"grasp=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm ===")

        # 0. 이웃 딸기 장애물 등록 (씬 위치 수신된 경우)
        self._register_neighbor_obstacles(straw)

        # 1. 그리퍼 열기
        self.get_logger().info("1 open gripper")
        self.set_held_strawberry_collision(False)
        if OPEN_GRIPPER_ON_PICK_START:
            self.call_trigger(self.cli_gripper_open)
            time.sleep(1.5)

        step = 2
        approach_start_joints = self.current_joints

        if USE_STAGING:
            self.get_logger().info(f"{step} staging (CuRobo 30cm)")
            ret = self.plan(self.current_joints, ee_s.tolist(), WALL_QUAT_WXYZ)
            if ret is None:
                self.get_logger().error("ABORT: staging plan failed")
                return
            # J1 branch 체크: 이상한 configuration이면 64-seed retry
            expected_j1 = np.pi / 2 + np.arctan2(-straw[0], straw[1])
            staging_j1 = float(ret[0][-1][0])
            j1_diff = abs(((staging_j1 - expected_j1 + np.pi) % (2 * np.pi)) - np.pi)
            if j1_diff > 2.0:
                self.get_logger().warn(
                    f"Staging J1 bad: {np.rad2deg(staging_j1):.1f}° (exp {np.rad2deg(expected_j1):.1f}°)"
                    f" → 64-seed retry")
                ret2 = self.plan(self.current_joints, ee_s.tolist(), WALL_QUAT_WXYZ, num_ik_seeds=64)
                if ret2 is not None:
                    ret = ret2
            if not self.execute_spline(*ret):
                self.get_logger().error("ABORT: staging exec failed")
                return
            approach_start_joints = ret[0][-1].tolist()
            step += 1

        if USE_APPROACH:
            # CuRobo: current/staging → approach (15cm)
            self.get_logger().info(f"{step} approach (CuRobo 15cm)")
            ret = self.plan(approach_start_joints, ee_a.tolist(), WALL_QUAT_WXYZ)
            if ret is None:
                self.get_logger().error("ABORT: approach plan failed")
                self.plan_to_fixed_joints_pose(self.current_joints, HOME_JOINTS_DEG, "home after approach fail")
                return
            if not self.execute_spline(*ret):
                self.get_logger().error("ABORT: approach exec failed")
                return
            approach_joints = ret[0][-1].tolist()
            step += 1
        else:
            self.get_logger().info(
                f"{step} direct grasp mode: skipping staging/approach segmentation")
            approach_joints = approach_start_joints

        if USE_CLOSE_CONFIRM:
            self.get_logger().info(
                f"{step} close-confirm (CuRobo {CLOSE_CONFIRM_OFFSET*100:.0f}cm before grasp)")
            ret = self.plan(approach_joints, ee_c.tolist(), WALL_QUAT_WXYZ)
            if ret is None:
                self.get_logger().error("ABORT: close-confirm plan failed")
                self._clear_neighbor_obstacles()
                self.pick_complete_pub.publish(Empty())
                return
            if not self.execute_spline(*ret):
                self.get_logger().error("ABORT: close-confirm exec failed")
                self._clear_neighbor_obstacles()
                self.pick_complete_pub.publish(Empty())
                return
            approach_joints = ret[0][-1].tolist()
            close_confirm_start = time.monotonic()
            time.sleep(CLOSE_CONFIRM_DWELL_SEC)
            if not self._wait_for_close_confirmation(straw, close_confirm_start):
                self.get_logger().warn(
                    "ABORT: close-view target not confirmed — retreat without grasp")
                ret_retreat = self.plan(approach_joints, ee_r.tolist(), WALL_QUAT_WXYZ)
                if ret_retreat is not None:
                    self.execute_spline(*ret_retreat)
                self._clear_neighbor_obstacles()
                self.pick_complete_pub.publish(Empty())
                return
            step += 1

        # CuRobo: approach → grasp
        n_offsets = len(ee_g_candidates)
        n_quats   = len(GRASP_QUAT_RETRY_DEG)
        self.get_logger().info(
            f"{step} grasp (CuRobo) — trying {n_offsets} offsets × {n_quats} quats "
            f"| target=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
            f"| start_J1={np.rad2deg(approach_joints[0]):.1f}°")
        ret = None
        used_grasp_offset = None
        used_grasp_quat_deg = None
        grasp_attempt = 0
        for grasp_offset, ee_g_try in ee_g_candidates:
            for quat_deg in GRASP_QUAT_RETRY_DEG:
                grasp_attempt += 1
                q_retry = quat_multiply_wxyz(
                    WALL_QUAT_WXYZ,
                    quat_from_axis_angle([1, 0, 0], np.deg2rad(quat_deg)),
                )
                ret = self.plan(approach_joints, ee_g_try.tolist(), q_retry,
                               num_ik_seeds=128)
                if ret is not None:
                    used_grasp_offset = grasp_offset
                    used_grasp_quat_deg = quat_deg
                    break
            if ret is not None:
                break

        if ret is not None:
            if not self.execute_spline(*ret):
                self.get_logger().error(
                    f"ABORT: grasp spline 실행 실패 "
                    f"(offset={used_grasp_offset:+.3f}m quat_x={used_grasp_quat_deg:+.1f}°)")
                return
            grasp_joints = ret[0][-1].tolist()
            self.get_logger().info(
                f"grasp OK — offset={used_grasp_offset:+.3f}m "
                f"quat_x={used_grasp_quat_deg:+.1f}° "
                f"(attempt {grasp_attempt}/{n_offsets * n_quats})")
        else:
            self.get_logger().error(
                f"ABORT: grasp 전체 실패 — {grasp_attempt}개 후보 모두 reject "
                f"(target=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
                f"start_J=[{', '.join(f'{np.rad2deg(v):.0f}' for v in approach_joints)}]°)")
            self._clear_neighbor_obstacles()
            self.pick_complete_pub.publish(Empty())
            return

        # 5. 그리퍼 닫기
        step += 1
        self.get_logger().info(f"{step} close gripper")
        self.soft_close_gripper()

        # 6. 파지 성공 판별
        grasp_ok = self._check_grasp()

        # 7. Retreat
        step += 1
        self.get_logger().info(f"{step} retreat (CuRobo)")
        ret = self.plan(grasp_joints, ee_r.tolist(), WALL_QUAT_WXYZ)
        if ret is not None:
            self.execute_spline(*ret)
            retreat_joints = ret[0][-1].tolist()
        else:
            self.get_logger().warn("Retreat plan failed — home으로 직행")
            ok, retreat_joints = self.plan_to_fixed_joints_pose(
                grasp_joints, HOME_JOINTS_DEG, "home after retreat fail")
            if not ok:
                self.get_logger().error("Home after retreat also failed — robot at grasp position")
                retreat_joints = grasp_joints

        if not grasp_ok:
            self.get_logger().warn("파지 실패 — VLA 이관 후 abort")
            self.set_held_strawberry_collision(False)
            self.call_trigger(self.cli_gripper_open)
            self.vla_request_pub.publish(msg)
            self.get_logger().warn(
                "HOME recovery skipped; future VLA retry policy will decide recovery motion")
            self._clear_neighbor_obstacles()
            self.pick_complete_pub.publish(Empty())
            return
        self.set_held_strawberry_collision(True)

        if not ENABLE_PLACE_SEQUENCE:
            self.get_logger().warn(
                "PLACE_DISABLED table collision risk — holding after retreat, no egg-tray move")
            self._clear_neighbor_obstacles()
            self.pick_complete_pub.publish(Empty())
            self.get_logger().info("=== PICK COMPLETE (NO PLACE) ===")
            return

        # 8. Place
        if USE_PRE_BIN_CLEAR:
            step += 1
            self.get_logger().info(f"{step} pre-bin clear (CuRobo)")
            ret = None
            used_clear_offset = None
            for clear_offset, ee_clear in ee_clear_candidates:
                if clear_offset != PRE_BIN_CLEAR_OFFSET:
                    self.get_logger().warn(f"pre-bin clear retry offset={clear_offset:.2f}m")
                ret = self.plan(retreat_joints, ee_clear.tolist(), WALL_QUAT_WXYZ)
                if ret is not None:
                    used_clear_offset = clear_offset
                    break
            if ret is None:
                self.get_logger().warn("Pre-bin clear plan failed — bin 이동 보류")
                return
            self.get_logger().info(f"pre-bin clear offset used={used_clear_offset:.2f}m")
            if not self.execute_spline(*ret):
                self.get_logger().error("ABORT: pre-bin clear exec failed — gripper 유지")
                return
            retreat_joints = ret[0][-1].tolist()

        # 항상 HOME 경유: retreat 후 J1이 크게 기울어진 상태에서 place로 직행하면
        # 벽 근접 궤도가 생김. HOME은 안전한 중립 경유지.
        step += 1
        self.get_logger().info(f"{step} → home (pre-place)")
        ok, transfer_joints = self.plan_to_fixed_joints_pose(
            retreat_joints, HOME_JOINTS_DEG, "pre-place home")
        if not ok:
            self.get_logger().error("ABORT: pre-place home failed — gripper 유지")
            return

        slot_idx, place_slot = self.current_place_slot()
        if place_slot is None:
            self.get_logger().error("ABORT: place 직전 사용 가능한 slot 없음 — gripper 유지")
            return
        slot_name = place_slot.get("name", f"slot{slot_idx}")

        if USE_TRAY_ENTRY_WAYPOINT and slot_idx and slot_idx > 0:
            entry_slot = self.tray_entry_slot()
            if entry_slot is not None:
                step += 1
                self.get_logger().info(
                    f"{step} → tray entry before {slot_name} (slot0 above)")
                ok, transfer_joints = self.move_to_place_target(
                    transfer_joints, entry_slot, "above", f"tray entry before {slot_name}")
                if not ok:
                    self.get_logger().error("ABORT: tray entry move failed — gripper 유지")
                    return

        step += 1
        self.get_logger().info(f"{step} → place {slot_name} above")
        ok, above_result_joints = self.move_to_place_target(
            transfer_joints, place_slot, "above", f"place {slot_name} above")
        if not ok:
            self.get_logger().error("ABORT: place above move failed — gripper 유지")
            return

        step += 1
        self.get_logger().info(f"{step} → place {slot_name} release")
        ok, release_result_joints = self.move_to_place_target(
            above_result_joints, place_slot, "release", f"place {slot_name} release")
        if not ok:
            self.get_logger().error("ABORT: place release move failed — gripper 유지")
            return
        time.sleep(0.5)
        self.call_trigger(self.cli_gripper_open)
        self.set_held_strawberry_collision(False)
        time.sleep(1.0)

        step += 1
        self.get_logger().info(f"{step} → place {slot_name} above retreat")
        ok, bin_joints = self.move_to_place_target(
            release_result_joints, place_slot, "above", f"place {slot_name} above retreat")
        if not ok:
            self.get_logger().error("Place above retreat failed after release")
            bin_joints = release_result_joints
        self.add_placed_strawberry_obstacle(slot_name, place_slot)
        self.advance_place_slot()

        # 9. Home 복귀
        step += 1
        self.get_logger().info(f"{step} → home")
        ok, _ = self.plan_to_fixed_joints_pose(bin_joints, HOME_JOINTS_DEG, "home")
        if not ok:
            self.get_logger().error("Home move failed after deposit")

        # 10. 이웃 장애물 해제 + 완료 신호
        self._clear_neighbor_obstacles()
        self.pick_complete_pub.publish(Empty())
        self.get_logger().info("=== PICK COMPLETE ===")


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
