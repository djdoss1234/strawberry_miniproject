#!/usr/bin/env python3
"""cuRobo Motion Planner Node for Doosan E0509

Pick sequence: open → grasp(CuRobo) → close → retreat(CuRobo) → pick_complete
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
from std_msgs.msg import Float64MultiArray, String, Empty
from std_srvs.srv import Trigger
from dsr_msgs2.srv import MoveSplineJoint, MoveJoint

from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState as CuroboJointState, RobotConfig
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.geom.types import WorldConfig, Cuboid, Sphere


# ── 파지 파라미터 ──────────────────────────────────────────────────────────────
GRASP_OFFSET        = +0.050   # TCP↔berry 거리 (m) — 조정 시 extension sphere 벽 간섭 주의
GRASP_RETRY_OFFSETS = [0.050, 0.065, 0.080, 0.100]
GRASP_Z_BIAS        = +0.025   # KP0 기준 Z 오프셋 (양수=위, 음수=아래)
RETREAT_OFFSET      = 0.36
RETREAT_UP_M        = 0.05     # retreat 시 Z 추가 (이웃 딸기 스침 방지)
NEIGHBOR_SPHERE_RADIUS_M = 0.030

GRIPPER_LEN      = 0.160       # ee_link → TCP (m)
WALL_SURFACE_Y_M = 0.672       # whiteboard 전면 Y — berry Y 클램핑 상한 (FK drift 보정)
WALL_UNIT        = np.array([-0.035, 0.996, -0.084])   # 티치펜던트 실측 (2026-05-18)
WALL_QUAT_WXYZ   = [0.548415, -0.439294, 0.424628, 0.570923]  # tool-Z ≈ WALL_UNIT
GRASP_QUAT_RETRY_DEG = [0.0, -8.0, 8.0, -15.0, 15.0]

CARTESIAN_PLAN_MAX_ATTEMPTS = 2
CARTESIAN_PLAN_TIMEOUT_SEC  = 1.2
DIRECT_GRASP_TARGET_X_RANGE_M = (-0.45, 0.45)

OPEN_GRIPPER_ON_PICK_START = True

# ── 고정 자세 ──────────────────────────────────────────────────────────────────
HOME_JOINTS_DEG = [88.0, -80.0, 130.0, 0.0, 20.0, -90.0]

# ── cuRobo 운용 한계 ───────────────────────────────────────────────────────────
OPERATIONAL_JOINT_LIMITS_DEG = [
    (-225.0, 225.0),   # J1
    (-95.0,   95.0),
    (-155.0, 155.0),
    (-270.0, 270.0),   # J4: SW scan pose=262.2° → ±175°로 제한 시 Doosan 360° 스핀 발생
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

        self.service_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.current_joints = None
        self._pick_busy = False
        self.static_cuboids = load_environment_cuboids()
        self.dynamic_cuboids = []
        self.neighbor_spheres: list = []
        self._scene_positions: list = []

        # ── cuRobo 초기화 ──────────────────────────────────────────────────────
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

        self.pick_complete_pub = self.create_publisher(Empty, "/dsr01/curobo/pick_complete", 10)

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
            f"  ENV_CUBOIDS={len(self.static_cuboids)}  "
            f"SELF_COLLISION={USE_CUROBO_SELF_COLLISION}")
        if os.path.exists(ENVIRONMENT_YAML):
            self.get_logger().info(f"  environment loaded: {ENVIRONMENT_YAML}")

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

    def _scene_cb(self, msg: Float64MultiArray) -> None:
        data = msg.data
        self._scene_positions = [
            np.array([data[i], data[i+1], data[i+2]])
            for i in range(0, len(data) - 2, 3)
        ]

    def _register_neighbor_obstacles(self, target_pos: np.ndarray) -> None:
        spheres = []
        for i, pos in enumerate(self._scene_positions):
            if np.linalg.norm(pos - target_pos) < 0.035:
                continue
            spheres.append(Sphere(
                name=f"neighbor_{i}",
                pose=[float(pos[0]), float(pos[1]), float(pos[2]), 1.0, 0.0, 0.0, 0.0],
                radius=NEIGHBOR_SPHERE_RADIUS_M,
            ))
        self.neighbor_spheres = spheres
        self.update_curobo_world("neighbor obstacles registered")
        self.get_logger().info(f"Registered {len(spheres)} neighbor sphere obstacle(s)")

    def _clear_neighbor_obstacles(self) -> None:
        if self.neighbor_spheres:
            self.neighbor_spheres = []
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
        if straw[0] > 0.25:
            return [-0.03, 0.0]
        return GRASP_RETRY_OFFSETS

    def set_held_strawberry_collision(self, enabled):
        if enabled:
            spheres = torch.tensor(
                [
                    [0.0, 0.0, 0.00,  0.026],
                    [0.0, 0.0, 0.025, 0.020],
                    [0.0, 0.0, 0.00, -100.0],
                    [0.0, 0.0, 0.00, -100.0],
                ],
                device="cuda:0", dtype=torch.float32,
            )
            self.motion_gen.attach_spheres_to_robot(
                sphere_tensor=spheres, link_name="attached_object")
        else:
            self.motion_gen.detach_object_from_robot()

    def call_trigger(self, client):
        if not client.wait_for_service(timeout_sec=3.0):
            return
        future = client.call_async(Trigger.Request())
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 10.0:
            time.sleep(0.1)

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
                delta_vals = np.abs(((vals - start_deg[joint_idx] + 180.0) % 360.0) - 180.0)
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

    def plan(self, start_joints, target_pos, target_quat_wxyz, num_ik_seeds=32):
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
        future = self.cli_spline.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)

        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error("Spline failed/timeout")
        return ok

    def plan_to_fixed_joints_pose(self, start_joints, target_joints_deg, label):
        """고정 joint 자세 이동 — cuRobo joint-space plan."""
        target_joints_rad = np.deg2rad(target_joints_deg).tolist()
        ret = self.plan_js(start_joints, target_joints_rad, label)
        if ret is not None and self.execute_spline(*ret):
            return True, ret[0][-1].tolist()
        self.get_logger().warn(f"{label} CuRobo joint-space failed")
        return False, start_joints

    # ── Pick 시퀀스 ────────────────────────────────────────────────────────────

    def pick_pose_cb(self, msg: PoseStamped):
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

        # Y 클램핑: berry는 벽 표면보다 뒤에 있을 수 없음 (FK drift 보정)
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
                f"ABORT: pick target x={raw_straw[0]*1000:.0f}mm outside "
                f"[{x_min*1000:.0f}, {x_max*1000:.0f}]mm")
            self.pick_complete_pub.publish(Empty())
            return

        grasp_retry_offsets = self.grasp_candidates_for_target(straw)
        ee_g_candidates = [
            (offset, straw - (offset + GRIPPER_LEN) * WALL_UNIT)
            for offset in grasp_retry_offsets
        ]
        ee_r = (straw - (RETREAT_OFFSET + GRIPPER_LEN) * WALL_UNIT
                + np.array([0.0, 0.0, RETREAT_UP_M]))

        self.get_logger().info(
            f"=== PICK 딸기 raw=({raw_straw[0]*1000:.0f},{raw_straw[1]*1000:.0f},{raw_straw[2]*1000:.0f})mm "
            f"grasp=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm ===")

        # 0. 이웃 딸기 장애물 등록
        self._register_neighbor_obstacles(straw)

        # 1. 그리퍼 열기
        self.get_logger().info("1 open gripper")
        self.set_held_strawberry_collision(False)
        if OPEN_GRIPPER_ON_PICK_START:
            self.call_trigger(self.cli_gripper_open)
            time.sleep(1.5)

        # 2. Grasp (cuRobo Cartesian)
        n_offsets = len(ee_g_candidates)
        n_quats   = len(GRASP_QUAT_RETRY_DEG)
        self.get_logger().info(
            f"2 grasp (CuRobo) — trying {n_offsets} offsets × {n_quats} quats "
            f"| target=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
            f"| start_J1={np.rad2deg(self.current_joints[0]):.1f}°")
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
                ret = self.plan(self.current_joints, ee_g_try.tolist(), q_retry,
                                num_ik_seeds=128)
                if ret is not None:
                    used_grasp_offset = grasp_offset
                    used_grasp_quat_deg = quat_deg
                    break
            if ret is not None:
                break

        if ret is None:
            self.get_logger().error(
                f"ABORT: grasp 전체 실패 — {grasp_attempt}개 후보 모두 reject "
                f"(target=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
                f"start_J=[{', '.join(f'{np.rad2deg(v):.0f}' for v in self.current_joints)}]°)")
            self._clear_neighbor_obstacles()
            self.pick_complete_pub.publish(Empty())
            return

        if not self.execute_spline(*ret):
            self.get_logger().error(
                f"ABORT: grasp spline 실행 실패 "
                f"(offset={used_grasp_offset:+.3f}m quat_x={used_grasp_quat_deg:+.1f}°)")
            self._clear_neighbor_obstacles()
            self.pick_complete_pub.publish(Empty())
            return

        grasp_joints = ret[0][-1].tolist()
        self.get_logger().info(
            f"grasp OK — offset={used_grasp_offset:+.3f}m "
            f"quat_x={used_grasp_quat_deg:+.1f}° "
            f"(attempt {grasp_attempt}/{n_offsets * n_quats})")

        # 3. 그리퍼 닫기
        self.get_logger().info("3 close gripper")
        self.call_trigger(self.cli_gripper_close)
        time.sleep(1.5)

        self.set_held_strawberry_collision(True)

        # 4. Retreat
        self.get_logger().info("4 retreat (CuRobo)")
        ret = self.plan(grasp_joints, ee_r.tolist(), WALL_QUAT_WXYZ)
        if ret is not None:
            self.execute_spline(*ret)
        else:
            self.get_logger().warn("Retreat plan failed — home으로 직행")
            ok, _ = self.plan_to_fixed_joints_pose(
                grasp_joints, HOME_JOINTS_DEG, "home after retreat fail")
            if not ok:
                self.get_logger().error("Home after retreat also failed — robot at grasp position")

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
