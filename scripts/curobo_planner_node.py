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
from curobo.geom.types import WorldConfig, Cuboid


# ── 딸기 접근 파라미터 ────────────────────────────────────────────────────────
APPROACH_OFFSET  = 0.15    # 딸기 앞 15cm (TCP 기준)
STAGING_EXTRA    = 0.15    # staging 추가 거리: approach보다 15cm 더 뒤
GRASP_OFFSET     = -0.050   # TCP를 딸기 중심보다 벽 방향으로 5cm 더 밀어 넣음
GRASP_RETRY_OFFSETS = [-0.05, -0.04, -0.03, 0.0]  # demo: 깊게 먼저 잡고 실패 시 얕게 재시도
RETREAT_OFFSET   = 0.36    # demo: 딸기/벽에서 더 빠져 place 이동 중 스침을 줄임
PRE_BIN_CLEAR_OFFSET = 0.42 # place 이동 전 벽에서 충분히 빠지는 clear 지점
PRE_BIN_CLEAR_RETRY_OFFSETS = [0.42, 0.36, 0.30]
GRASP_Z_BIAS     = -0.025  # 검출 중심보다 30mm 낮게 파지
USE_STAGING      = False   # True: 30cm staging → 15cm approach → grasp
USE_PRE_BIN_CLEAR = False  # demo: retreat 후 place로 바로 이동해 불필요한 우회/딸기 접촉을 줄임
USE_CUROBO_FIXED_POSES = True  # True: home/place 고정 joint 자세도 cuRobo joint-space로 계획
USE_MOVEJ_FOR_DEMO_PLACE = True # demo: 계란판 위 짧은 release/home만 MoveJoint, 큰 이동은 cuRobo 유지
ALLOW_MOVEJ_FALLBACK = False   # True: cuRobo 실패 시 MoveJoint로 후퇴. 100% cuRobo 검증 중에는 False 권장
USE_CUROBO_SELF_COLLISION = False  # 현재 coarse sphere 모델은 정상 자세도 self-collision으로 오검출함
USE_PLACED_STRAWBERRY_OBSTACLES = False  # demo: 놓인 딸기 obstacle은 영상 성공 후 다시 켜서 검증
MAX_SPLINE_POINTS = 12         # Doosan spline point가 너무 많으면 실기에서 뚝뚝 끊겨 보임
SPLINE_TIME_SCALE = 0.75       # demo: 계획보다 살짝 빠르게 실행
SPLINE_MIN_TIME = 0.5
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
    (-170.0, 170.0),   # J4: 과도한 wrist flip 금지
    (-130.0, 130.0),
    (-225.0, 225.0),   # J6: wrist wrap은 180도 근처까지 허용
]

GRIPPER_LEN      = 0.160   # ee_link → TCP 거리 (m)
WALL_UNIT        = np.array([-0.035, 0.996, -0.084])   # 티치펜던트 실측 (2026-05-18)
WALL_QUAT_WXYZ   = [0.548415, -0.439294, 0.424628, 0.570923]  # ee_link [w,x,y,z] (2026-05-18)
GRASP_QUAT_RETRY_DEG = [0.0]  # 현장 운용 기본: orientation 고정, 긴 retry 금지

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
USE_SOFT_CLOSE    = True
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
            collision_cache={"obb": 30, "mesh": 10},
            use_cuda_graph=False,
            self_collision_check=USE_CUROBO_SELF_COLLISION,
            self_collision_opt=USE_CUROBO_SELF_COLLISION,
        )
        self.motion_gen = MotionGen(motion_gen_cfg)
        self.motion_gen.warmup(warmup_js_trajopt=False)
        self.motion_gen.detach_object_from_robot()
        self.get_logger().info("cuRobo MotionGen warmed up!")

        # ── ROS2 인터페이스 ───────────────────────────────────────────────────
        self.create_subscription(JointState, "/dsr01/joint_states", self.joint_state_cb, 10)
        self.create_subscription(PoseStamped, "/dsr01/curobo/target_pose", self.target_pose_cb, 10)
        self.create_subscription(PoseStamped, "/dsr01/curobo/pick_pose", self.pick_pose_cb, 10)
        self.create_subscription(String, "/dsr01/curobo/obstacles", self.obstacles_cb, 10)
        self.create_subscription(Int32, "/dsr01/gripper/stroke", self._stroke_cb, 10)

        self.pick_complete_pub = self.create_publisher(Empty, "/dsr01/curobo/pick_complete", 10)
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
        self.motion_gen.update_world(WorldConfig(cuboid=cuboids))
        self.get_logger().info(
            f"World updated ({reason}): static={len(self.static_cuboids)} "
            f"dynamic={len(self.dynamic_cuboids)} placed={len(self.placed_cuboids)}")

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
            start_state, target_pose, MotionGenPlanConfig(num_ik_seeds=num_ik_seeds)
        )
        dt = (time.time() - t0) * 1000

        if result.success.item():
            traj = result.get_interpolated_plan().position.cpu().numpy()
            if not self.trajectory_in_operational_limits(traj, "Cartesian plan"):
                return None
            motion_time = float(result.motion_time.item())
            self.get_logger().info(
                f"Plan OK {dt:.0f}ms {traj.shape[0]}pts {motion_time:.2f}s | "
                f"goal={[f'{v*1000:.0f}' for v in target_pos]}mm")
            return traj, motion_time
        else:
            status = getattr(result, "status", "?")
            self.get_logger().error(
                f"Plan FAIL {dt:.0f}ms | status={status} | "
                f"goal={[f'{v*1000:.0f}' for v in target_pos]}mm")
            if "INVALID_START_STATE_WORLD_COLLISION" in str(status):
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
            if not self.trajectory_in_operational_limits(traj, label):
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

    # ── Pick 시퀀스 ───────────────────────────────────────────────────────────

    def pick_pose_cb(self, msg: PoseStamped):
        """open → approach → grasp → close → [check] → retreat → place → home"""
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
        next_slot_idx, next_place_slot = self.current_place_slot()
        if next_place_slot is None:
            self.get_logger().error("ABORT: 사용 가능한 place slot 없음 — 계란판 slot 티칭/초기화 필요")
            self.pick_complete_pub.publish(Empty())
            return

        p = msg.pose.position
        raw_straw = np.array([p.x, p.y, max(p.z, 0.05)])
        straw = raw_straw + np.array([0.0, 0.0, GRASP_Z_BIAS])
        straw[2] = max(straw[2], 0.05)

        # CuRobo ee_link 목표 위치
        ee_s = straw - (APPROACH_OFFSET + STAGING_EXTRA + GRIPPER_LEN) * WALL_UNIT
        ee_a = straw - (APPROACH_OFFSET + GRIPPER_LEN) * WALL_UNIT
        grasp_retry_offsets = self.grasp_candidates_for_target(straw)
        ee_g = straw - (grasp_retry_offsets[0] + GRIPPER_LEN) * WALL_UNIT
        ee_g_candidates = [
            (offset, straw - (offset + GRIPPER_LEN) * WALL_UNIT)
            for offset in grasp_retry_offsets
        ]
        ee_r = straw - (RETREAT_OFFSET  + GRIPPER_LEN) * WALL_UNIT
        ee_clear_candidates = [
            (offset, straw - (offset + GRIPPER_LEN) * WALL_UNIT)
            for offset in PRE_BIN_CLEAR_RETRY_OFFSETS
        ]

        self.get_logger().info(
            f"=== PICK 딸기 raw=({raw_straw[0]*1000:.0f},{raw_straw[1]*1000:.0f},{raw_straw[2]*1000:.0f})mm "
            f"grasp=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm ===")

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

        # CuRobo: approach → grasp
        self.get_logger().info(f"{step} grasp (CuRobo)")
        ret = None
        used_grasp_offset = None
        used_grasp_quat_deg = None
        for grasp_offset, ee_g_try in ee_g_candidates:
            if grasp_offset != GRASP_OFFSET:
                self.get_logger().warn(f"grasp retry offset={grasp_offset:+.3f}m")
            for quat_deg in GRASP_QUAT_RETRY_DEG:
                q_retry = quat_multiply_wxyz(
                    WALL_QUAT_WXYZ,
                    quat_from_axis_angle([1, 0, 0], np.deg2rad(quat_deg)),
                )
                if quat_deg != 0.0:
                    self.get_logger().warn(f"grasp retry quat_x={quat_deg:+.1f}deg")
                ret = self.plan(approach_joints, ee_g_try.tolist(), q_retry)
                if ret is not None:
                    used_grasp_offset = grasp_offset
                    used_grasp_quat_deg = quat_deg
                    break
            if ret is not None:
                break

        if ret is not None:
            if not self.execute_spline(*ret):
                self.get_logger().error("ABORT: grasp exec failed")
                return
            grasp_joints = ret[0][-1].tolist()
            self.get_logger().info(
                f"grasp offset used={used_grasp_offset:+.3f}m "
                f"quat_x={used_grasp_quat_deg:+.1f}deg")
        else:
            self.get_logger().error("ABORT: grasp CuRobo plan failed for all candidates")
            ret2 = self.plan(approach_joints, ee_r.tolist(), WALL_QUAT_WXYZ)
            if ret2 is not None:
                self.execute_spline(*ret2)
                approach_joints = ret2[0][-1].tolist()
            self.plan_to_fixed_joints_pose(approach_joints, HOME_JOINTS_DEG, "home after grasp fail")
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
            retreat_joints = grasp_joints

        if not grasp_ok:
            self.get_logger().warn("파지 실패 — abort")
            self.set_held_strawberry_collision(False)
            self.call_trigger(self.cli_gripper_open)
            self.plan_to_fixed_joints_pose(retreat_joints, HOME_JOINTS_DEG, "home after grasp check fail")
            self.pick_complete_pub.publish(Empty())
            return
        self.set_held_strawberry_collision(True)

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

        transfer_joints = retreat_joints

        if USE_LEFT_SAFE_TRANSFER and raw_straw[0] < LEFT_SAFE_TRANSFER_X_MAX:
            step += 1
            self.get_logger().info(f"{step} → left safe transfer")
            ok, transfer_joints = self.plan_to_fixed_joints_pose(
                transfer_joints, LEFT_SAFE_TRANSFER_JOINTS_DEG, "left safe transfer")
            if not ok:
                self.get_logger().error("ABORT: left safe transfer failed — gripper 유지")
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

        # 10. 완료 신호
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
