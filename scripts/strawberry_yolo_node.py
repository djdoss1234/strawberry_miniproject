#!/usr/bin/env python3
"""
Strawberry YOLO Pick Node — 캘리브레이션 FK 직접 사용 버전

변환 파이프라인 (캘리브레이션 코드와 동일):
  pt_cam → T_cam2gripper → T_gripper2base(FK+TCP_fixed) → pt_base

TF2, static_transform_publisher 불필요.
"""

import os
import time
import threading
import json
from datetime import datetime
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
import pyrealsense2 as rs
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as ScipyR
from dsr_msgs2.srv import MoveJoint

HOME_JOINTS_DEG   = [88.0, -80.0, 130.0, 0.0, 20.0, -90.0]
MAX_GHOST_FRAMES  = 10   # 트래킹 놓쳤을 때 ID 보존 프레임 수 (~0.33s @ 30fps)
YOLO_CONF         = 0.25

# ── ripe 후보 필터 ───────────────────────────────────────────────────────────
USE_RIPE_FILTER     = True
RIPE_RED_RATIO_MIN  = 0.28
RIPE_STRONG_RED_RATIO_MIN = 0.12
RIPE_SAT_MEAN_MIN = 105.0
RIPE_STABLE_FRAMES  = 1
UNRIPE_CLASS_KEYWORDS = ("unripe", "green", "immature")
RED_HSV_RANGES = (
    # 시연용 중간값: 너무 옅은 핑크는 줄이되 정상 빨간 딸기는 통과시킨다.
    (np.array([0, 90, 50], dtype=np.uint8), np.array([12, 255, 255], dtype=np.uint8)),
    (np.array([168, 90, 50], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8)),
)
STRONG_RED_HSV_RANGES = (
    (np.array([0, 120, 60], dtype=np.uint8), np.array([12, 255, 255], dtype=np.uint8)),
    (np.array([168, 120, 60], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8)),
)

MODEL_PATH = os.path.expanduser(
    "~/doosan_ws/src/e0509_gripper_description/models/best.pt"
)
CALIB_NPZ = os.path.expanduser(
    "~/doosan_ws/src/e0509_gripper_description/config/calibration_eye_in_hand_1.npz"
)
LOG_ROOT = os.path.expanduser(
    "~/doosan_ws/src/e0509_gripper_description/logs/pick_attempts"
)

# ── E0509 FK (캘리브레이션 코드와 동일) ───────────────────────────────────────
def _T(xyz, rpy, q=0.0):
    M = np.eye(4)
    M[:3, 3] = xyz
    R_fixed = ScipyR.from_euler('xyz', rpy).as_matrix()
    R_joint = ScipyR.from_euler('z', q).as_matrix()
    M[:3, :3] = R_fixed @ R_joint
    return M

def e0509_fk(q_rad):
    """캘리브레이션 코드와 동일한 FK (TCP fixed transform 포함)"""
    T = np.eye(4)
    T = T @ _T([0,      0,       0.2045], [0,          0,           0      ], q_rad[0])
    T = T @ _T([0,      0,       0     ], [0,         -np.pi/2,    -np.pi/2], q_rad[1])
    T = T @ _T([0.373,  0,       0     ], [0,          0,           np.pi/2], q_rad[2])
    T = T @ _T([0,     -0.373,   0     ], [np.pi/2,    0,           0      ], q_rad[3])
    T = T @ _T([0,      0,       0     ], [-np.pi/2,   0,           0      ], q_rad[4])
    T = T @ _T([0,     -0.1725,  0     ], [np.pi/2,    0,           0      ], q_rad[5])
    T = T @ _T([0,      0,       0     ], [np.pi,     -np.pi/2,     0      ])  # TCP fixed
    return T


def red_mask_bgr(image_bgr):
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for lower, upper in RED_HSV_RANGES:
        part = cv2.inRange(hsv, lower, upper)
        mask = part if mask is None else (mask | part)
    return mask


def hsv_mask_bgr(image_bgr, ranges):
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for lower, upper in ranges:
        part = cv2.inRange(hsv, lower, upper)
        mask = part if mask is None else (mask | part)
    return mask, hsv


def depth_target_from_bbox(depth_frame, image_bgr, box_xyxy):
    """딸기 표면 depth 추정.

    bbox 중심 median은 뒤판 depth를 잡기 쉬워서, 빨간 픽셀의 가까운 depth를 우선 사용한다.
    반환: (u, v, depth[m])
    """
    h, w = image_bgr.shape[:2]
    x0, y0, x1, y1 = [int(v) for v in box_xyxy]
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w - 1, x1), min(h - 1, y1)
    if x1 <= x0 or y1 <= y0:
        return None

    crop = image_bgr[y0:y1, x0:x1]
    red_mask = red_mask_bgr(crop)

    samples = []
    red_samples = []
    for v in range(y0, y1, 2):
        for u in range(x0, x1, 2):
            d = depth_frame.get_distance(u, v)
            if 0.05 < d < 3.0:
                samples.append((u, v, d))
                if red_mask[v - y0, u - x0] > 0:
                    red_samples.append((u, v, d))

    use_samples = red_samples if len(red_samples) >= 20 else samples
    if not use_samples:
        return None

    depths = np.array([s[2] for s in use_samples], dtype=np.float32)
    target_d = float(np.percentile(depths, 20))
    near = [s for s in use_samples if abs(s[2] - target_d) < 0.03]
    if not near:
        near = use_samples

    u = int(np.median([s[0] for s in near]))
    v = int(np.median([s[1] for s in near]))
    return u, v, target_d


def red_ratio_in_bbox(image_bgr, box_xyxy):
    """bbox 내부 빨간 픽셀 비율. YOLO class가 단일 strawberry일 때 ripe score로 사용."""
    h, w = image_bgr.shape[:2]
    x0, y0, x1, y1 = [int(v) for v in box_xyxy]
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w - 1, x1), min(h - 1, y1)
    if x1 <= x0 or y1 <= y0:
        return 0.0

    crop = image_bgr[y0:y1, x0:x1]
    mask = red_mask_bgr(crop)
    return float(np.count_nonzero(mask)) / float(mask.size)


def ripe_metrics_in_bbox(image_bgr, box_xyxy):
    h, w = image_bgr.shape[:2]
    x0, y0, x1, y1 = [int(v) for v in box_xyxy]
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w - 1, x1), min(h - 1, y1)
    if x1 <= x0 or y1 <= y0:
        return {"red_ratio": 0.0, "strong_red_ratio": 0.0, "sat_mean": 0.0}

    crop = image_bgr[y0:y1, x0:x1]
    red_mask = red_mask_bgr(crop)
    strong_mask, hsv = hsv_mask_bgr(crop, STRONG_RED_HSV_RANGES)

    b, g, r = cv2.split(crop)
    dominance = ((r.astype(np.float32) > g.astype(np.float32) * 1.20) &
                 (r.astype(np.float32) > b.astype(np.float32) * 1.20))
    strong_mask = strong_mask & dominance.astype(np.uint8) * 255

    red_pixels = red_mask > 0
    sat_mean = float(np.mean(hsv[:, :, 1][red_pixels])) if np.any(red_pixels) else 0.0
    total = float(red_mask.size)
    return {
        "red_ratio": float(np.count_nonzero(red_mask)) / total,
        "strong_red_ratio": float(np.count_nonzero(strong_mask)) / total,
        "sat_mean": sat_mean,
    }


def make_pick_msg(clock, xyz):
    msg = PoseStamped()
    msg.header.frame_id = "base_link"
    msg.header.stamp = clock.now().to_msg()
    msg.pose.position.x = float(xyz[0])
    msg.pose.position.y = float(xyz[1])
    msg.pose.position.z = float(xyz[2])
    return msg


def class_name_from_box(model_names, box):
    if box.cls is None:
        return "unknown"
    cls_id = int(box.cls[0].item())
    if isinstance(model_names, dict):
        return str(model_names.get(cls_id, cls_id))
    if isinstance(model_names, (list, tuple)) and cls_id < len(model_names):
        return str(model_names[cls_id])
    return str(cls_id)


def ripe_filter_reason(class_name, metrics):
    """Return None when pickable; otherwise return the skip reason."""
    name = class_name.lower()
    if any(k in name for k in UNRIPE_CLASS_KEYWORDS):
        return "unripe-class"
    if metrics["red_ratio"] < RIPE_RED_RATIO_MIN:
        return "low-red"
    if metrics["strong_red_ratio"] < RIPE_STRONG_RED_RATIO_MIN:
        return "weak-red"
    if metrics["sat_mean"] < RIPE_SAT_MEAN_MIN:
        return "low-sat"
    return None


class StrawberryYoloNode(Node):

    JOINT_NAMES = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]

    def __init__(self):
        super().__init__("strawberry_yolo_node")

        # ── 캘리브레이션 로드 ──────────────────────────────────────────────────
        self.get_logger().info(f"Loading calibration: {CALIB_NPZ}")
        calib = np.load(CALIB_NPZ)
        self.T_cam2gripper = calib['T_cam_to_gripper']  # (4,4)
        self.get_logger().info(
            f"T_cam2gripper translation(mm): {self.T_cam2gripper[:3,3]*1000}"
        )

        # ── 조인트 상태 ────────────────────────────────────────────────────────
        self.current_joints = None

        # ── YOLO ──────────────────────────────────────────────────────────────
        self.get_logger().info(f"Loading YOLO: {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f"YOLO classes: {self.model.names}")
        self.get_logger().info(
            f"Ripe filter: {'ON' if USE_RIPE_FILTER else 'OFF'} "
            f"(red_ratio >= {RIPE_RED_RATIO_MIN:.2f})")

        # ── RealSense ─────────────────────────────────────────────────────────
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)
        self.get_logger().info("RealSense started.")

        # ── ROS2 ──────────────────────────────────────────────────────────────
        self.joint_sub = self.create_subscription(
            JointState, "/dsr01/joint_states", self.joint_cb, 10)
        self.pick_pub = self.create_publisher(
            PoseStamped, "/dsr01/curobo/pick_pose", 10)
        self.cli_movej = self.create_client(MoveJoint, "/dsr01/motion/move_joint")
        self._homing = False

        self.timer = self.create_timer(1.0 / 30.0, self.loop)
        self.candidates = []      # [(msg, bbox, area, ripe_score, cls_name, stable_count), ...]
        self.selected_idx = 0
        self._tracks = []         # [[x,y,z, cx,cy, bx0,by0,bx1,by1, track_id], ...]
        self._next_tid = 0
        self._candidate_tids = []
        self._alpha = 0.05
        self._alpha_px = 0.2
        self.locked_pos = None
        self.locked_tid = None
        self._last_vis_img = None
        self._last_attempt = None

        # 자동 연속 수확
        self.auto_mode = False
        self.pending_auto_pick = False
        self._auto_blocked_tids = set()
        self._demo_blocked_tids = set()
        self.create_subscription(Empty, "/dsr01/curobo/pick_complete", self._pick_complete_cb, 10)

        self.get_logger().info(
            "Strawberry YOLO Node Ready!  '1~9'=Lock  's'=Pick  "
            "'x'=Block  'y/n/d/m'=Label  'a'=Auto  'q'=Quit")

    # ── 조인트 콜백 ───────────────────────────────────────────────────────────
    def joint_cb(self, msg: JointState):
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        try:
            self.current_joints = [jmap[n] for n in self.JOINT_NAMES]
        except KeyError:
            pass

    def _pick_complete_cb(self, msg):
        self._log_pick_result("pick_complete")
        if self.auto_mode and self._last_attempt is not None:
            candidate = self._last_attempt.get("candidate") or {}
            tid = candidate.get("track_id", self._last_attempt.get("locked_tid"))
            if tid is not None:
                self._auto_blocked_tids.add(int(tid))
                self.get_logger().info(f"Auto block tid={tid} after pick_complete")
        if not self.auto_mode:
            return
        self.locked_pos = None
        self.locked_tid = None
        self.pending_auto_pick = True
        self.get_logger().info("Pick complete → 다음 딸기 자동 선택 대기")

    def _go_home(self):
        if not self.cli_movej.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveJoint service not available")
            self._homing = False
            return
        req = MoveJoint.Request()
        req.pos = HOME_JOINTS_DEG
        req.vel = 30.0
        req.acc = 30.0
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        future = self.cli_movej.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 30.0:
            time.sleep(0.1)
        if future.done() and future.result() and future.result().success:
            self.get_logger().info("Home reached!")
        else:
            self.get_logger().error("Home move failed!")
        self._homing = False

    # ── 카메라 → base_link 변환 (캘리브레이션 코드와 동일) ────────────────────
    def cam_to_base(self, pt_cam_xyz):
        if self.current_joints is None:
            return None
        # T_gripper2base: FK + TCP fixed (캘리브레이션 코드와 동일)
        T_gripper2base = e0509_fk(self.current_joints)
        T_total = T_gripper2base @ self.T_cam2gripper
        pt_h = np.array([*pt_cam_xyz, 1.0])
        return (T_total @ pt_h)[:3]

    def _candidate_meta(self, idx):
        if idx is None or idx < 0 or idx >= len(self.candidates):
            return {}
        msg, b, area, ripe_score, cls_name, stable_count = self.candidates[idx]
        p = msg.pose.position
        tid = self._candidate_tids[idx] if idx < len(self._candidate_tids) else None
        return {
            "candidate_idx": int(idx),
            "track_id": int(tid) if tid is not None else None,
            "class_name": str(cls_name),
            "red_ratio": float(ripe_score),
            "stable_count": int(stable_count),
            "bbox_xyxy": [float(v) for v in b],
            "area": float(area),
            "candidate_xyz": [float(p.x), float(p.y), float(p.z)],
        }

    def _log_pick_attempt(self, mode, target_msg, candidate_idx=None):
        now = datetime.now()
        day_dir = os.path.join(LOG_ROOT, now.strftime("%Y-%m-%d"))
        image_dir = os.path.join(day_dir, "images")
        os.makedirs(image_dir, exist_ok=True)

        stamp = now.strftime("%H%M%S_%f")
        meta = self._candidate_meta(candidate_idx)
        tid = meta.get("track_id", self.locked_tid)
        image_name = f"{stamp}_tid{tid if tid is not None else 'none'}_{mode}.jpg"
        image_path = os.path.join(image_dir, image_name)
        if self._last_vis_img is not None:
            cv2.imwrite(image_path, self._last_vis_img)
        else:
            image_path = None

        p = target_msg.pose.position
        record = {
            "timestamp": now.isoformat(timespec="milliseconds"),
            "mode": mode,
            "target_xyz": [float(p.x), float(p.y), float(p.z)],
            "locked_tid": int(self.locked_tid) if self.locked_tid is not None else None,
            "locked_pos": [float(v) for v in self.locked_pos] if self.locked_pos is not None else None,
            "candidate": meta,
            "image": image_path,
            "ripe_filter": {
                "enabled": bool(USE_RIPE_FILTER),
                "red_ratio_min": float(RIPE_RED_RATIO_MIN),
                "strong_red_ratio_min": float(RIPE_STRONG_RED_RATIO_MIN),
                "sat_mean_min": float(RIPE_SAT_MEAN_MIN),
                "stable_frames": int(RIPE_STABLE_FRAMES),
            },
        }
        jsonl_path = os.path.join(day_dir, "attempts.jsonl")
        with open(jsonl_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._last_attempt = record
        self.get_logger().info(f"Pick log saved: {jsonl_path}")

    def _log_pick_result(self, event):
        now = datetime.now()
        day_dir = os.path.join(LOG_ROOT, now.strftime("%Y-%m-%d"))
        os.makedirs(day_dir, exist_ok=True)
        record = {
            "timestamp": now.isoformat(timespec="milliseconds"),
            "event": event,
            "result": "sequence_complete",
            "last_attempt": self._last_attempt,
        }
        jsonl_path = os.path.join(day_dir, "attempts.jsonl")
        with open(jsonl_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.get_logger().info(f"Pick result logged: {event}")

    def _log_manual_label(self, label):
        now = datetime.now()
        day_dir = os.path.join(LOG_ROOT, now.strftime("%Y-%m-%d"))
        os.makedirs(day_dir, exist_ok=True)
        record = {
            "timestamp": now.isoformat(timespec="milliseconds"),
            "event": "manual_label",
            "label": label,
            "last_attempt": self._last_attempt,
        }
        jsonl_path = os.path.join(day_dir, "attempts.jsonl")
        with open(jsonl_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.get_logger().info(f"Manual label saved: {label}")

    # ── 메인 루프 ─────────────────────────────────────────────────────────────
    def loop(self):
        try:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f:
                return

            img = np.asanyarray(color_f.get_data())
            intr = depth_f.profile.as_video_stream_profile().intrinsics

            # 조인트 상태 표시
            if self.current_joints is not None:
                cv2.putText(img, "Joint OK", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,100), 1)
            else:
                cv2.putText(img, "NO JOINT STATE", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                cv2.imshow("Strawberry Detection", img)
                cv2.waitKey(1)
                return

            results = self.model(img, conf=YOLO_CONF, verbose=False)

            candidates = []

            for r in results:
                for box in r.boxes:
                    b = box.xyxy[0].cpu().numpy()
                    cls_name = class_name_from_box(self.model.names, box)
                    ripe_metrics = ripe_metrics_in_bbox(img, b)
                    ripe_score = ripe_metrics["red_ratio"]

                    area = (b[2]-b[0]) * (b[3]-b[1])

                    depth_target = depth_target_from_bbox(depth_f, img, b)
                    if depth_target is None:
                        cv2.rectangle(img,(int(b[0]),int(b[1])),(int(b[2]),int(b[3])),(0,0,255),1)
                        continue
                    target_u, target_v, depth = depth_target
                    cv2.circle(img, (target_u, target_v), 3, (255, 0, 255), -1)

                    pt_cam = rs.rs2_deproject_pixel_to_point(intr, [target_u, target_v], depth)
                    pt_base = self.cam_to_base(pt_cam)
                    if pt_base is None:
                        continue

                    px, py, pz = pt_base
                    msg = PoseStamped()
                    msg.header.frame_id = "base_link"
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.pose.position.x = float(px)
                    msg.pose.position.y = float(py)
                    msg.pose.position.z = float(pz)
                    candidates.append((msg, b, area, ripe_score, cls_name, ripe_metrics))

            # 면적 큰 순으로 정렬
            candidates.sort(key=lambda c: c[2], reverse=True)

            # EMA 트래킹 + 고스트 트랙 보존 (10프레임간 ID 유지)
            # track 포맷: [sx,sy,sz, scx,scy, sb0,sb1,sb2,sb3, tid, frames_since_seen]
            new_tracks = []
            new_candidates_with_tid = []
            skipped_candidates = []
            used_old_tracks = set()
            for msg, b, area, ripe_score, cls_name, ripe_metrics in candidates:
                p = msg.pose.position
                cx_raw = (b[0] + b[2]) / 2
                cy_raw = (b[1] + b[3]) / 2
                # 가장 가까운 기존 트랙 매칭 (nearest, not first)
                best_dist, matched = 0.10, None
                for track in self._tracks:
                    if id(track) in used_old_tracks:
                        continue
                    dist = ((p.x-track[0])**2 + (p.y-track[1])**2 + (p.z-track[2])**2) ** 0.5
                    if dist < best_dist:
                        best_dist, matched = dist, track
                if matched is not None:
                    used_old_tracks.add(id(matched))
                    a3 = self._alpha
                    ap = self._alpha_px
                    sx = matched[0] + a3 * (p.x - matched[0])
                    sy = matched[1] + a3 * (p.y - matched[1])
                    sz = matched[2] + a3 * (p.z - matched[2])
                    scx = matched[3] + ap * (cx_raw - matched[3])
                    scy = matched[4] + ap * (cy_raw - matched[4])
                    sb = [
                        matched[5] + ap * (b[0] - matched[5]),
                        matched[6] + ap * (b[1] - matched[6]),
                        matched[7] + ap * (b[2] - matched[7]),
                        matched[8] + ap * (b[3] - matched[8]),
                    ]
                    tid = matched[9]
                    old_stable = matched[11] if len(matched) > 11 else 0
                else:
                    sx, sy, sz = p.x, p.y, p.z
                    scx, scy = cx_raw, cy_raw
                    sb = list(b)
                    tid = self._next_tid
                    self._next_tid += 1

                    old_stable = 0

                skip_reason = ripe_filter_reason(cls_name, ripe_metrics) if USE_RIPE_FILTER else None
                if int(tid) in self._demo_blocked_tids:
                    skip_reason = "demo-blocked"
                stable_count = old_stable + 1 if skip_reason is None else 0
                new_tracks.append([sx, sy, sz, scx, scy, sb[0], sb[1], sb[2], sb[3],
                                   tid, 0, stable_count])
                msg.pose.position.x = sx
                msg.pose.position.y = sy
                msg.pose.position.z = sz

                if stable_count >= RIPE_STABLE_FRAMES:
                    new_candidates_with_tid.append((msg, sb, area, ripe_score, cls_name, stable_count, tid))
                else:
                    skipped_candidates.append((sb, cls_name, ripe_score, skip_reason, stable_count))

            # 이번 프레임에 매칭 안 된 트랙 → 고스트로 보존 (ID 연속성 유지)
            for track in self._tracks:
                if id(track) in used_old_tracks:
                    continue
                fseen = (track[10] + 1) if len(track) > 10 else 1
                if fseen < MAX_GHOST_FRAMES:
                    ghost = list(track)
                    if len(ghost) > 10:
                        ghost[10] = fseen
                    else:
                        ghost.append(fseen)
                    if len(ghost) > 11:
                        ghost[11] = 0
                    new_tracks.append(ghost)

            self._tracks = new_tracks
            self._candidate_tids = [t[6] for t in new_candidates_with_tid]
            candidates = [(msg, sb, area, ripe_score, cls_name, stable_count)
                          for msg, sb, area, ripe_score, cls_name, stable_count, _ in new_candidates_with_tid]

            self.candidates = candidates  # already smoothed
            if self.candidates:
                self.selected_idx = min(self.selected_idx, len(self.candidates) - 1)
            else:
                self.selected_idx = 0

            # ── Lock 추적: track_id 기반으로 locked_idx 결정 ──
            # 번호가 바뀌어도 ID가 같으면 같은 딸기로 인식
            locked_idx = None
            if self.locked_tid is not None:
                for i, tid in enumerate(self._candidate_tids):
                    if tid == self.locked_tid:
                        locked_idx = i
                        break

            # ── 시각화 ────────────────────────────────────────────────────────
            for b, cls_name, ripe_score, skip_reason, stable_count in skipped_candidates:
                x0, y0, x1, y1 = [int(v) for v in b]
                reason = skip_reason if skip_reason is not None else "unstable"
                cv2.rectangle(img, (x0, y0), (x1, y1), (80, 80, 80), 1)
                cv2.putText(img, f"skip {cls_name} {reason} R:{ripe_score:.2f} S:{stable_count}/{RIPE_STABLE_FRAMES}",
                            (x0, max(15, y0 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 120, 120), 1)

            for i, (msg, b, area, ripe_score, cls_name, stable_count) in enumerate(self.candidates):
                p = msg.pose.position

                if self.locked_tid is not None:
                    is_locked = (i == locked_idx)
                    color = (0, 80, 255) if is_locked else (60, 60, 60)
                    thickness = 3 if is_locked else 1
                    if is_locked:
                        lp = self.locked_pos
                        cv2.putText(img, f"LOCKED ({lp[0]:.3f},{lp[1]:.3f},{lp[2]:.3f})",
                                    (int(b[0]), int(b[3]) + 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 80, 255), 2)
                else:
                    is_selected = (i == self.selected_idx)
                    color = (0, 255, 255) if is_selected else (0, 255, 0)
                    thickness = 3 if is_selected else 1

                cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), color, thickness)
                x_text = int(max(4, min(b[0], img.shape[1] - 250)))
                y_text = int(max(28, b[1] - 24))
                label1 = f"[{i+1}] {cls_name} Z:{p.z:.3f} R:{ripe_score:.2f} S:{stable_count}"
                label2 = f"X:{p.x:.3f} Y:{p.y:.3f}"
                cv2.putText(img, label1, (x_text, y_text),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
                cv2.putText(img, label2, (x_text, y_text + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)

            n = len(self.candidates)
            auto_str = " [AUTO]" if self.auto_mode else ""
            if self.locked_tid is not None:
                status = "LOCKED" if locked_idx is not None else "LOST!"
                hint = f"[{status}]{auto_str}  s:Pick  x:Block  y/n/d/m:Label  u:Unlock  a:Auto  h:Home  q:Quit"
            else:
                hint = f"1~{min(n,9)}:Lock  s:Pick  x:Block  y/n/d/m:Label  a:Auto  h:Home  q:Quit" if n else "x:Block  y/n/d/m:Label  a:Auto  h:Home  q:Quit"
            cv2.putText(img, hint, (10, 460),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            # 4-cell grid overlay (NW/NE/SW/SE)
            h, w = img.shape[:2]
            cx, cy = w // 2, h // 2
            cv2.line(img, (cx, 0), (cx, h), (0, 255, 0), 1)
            cv2.line(img, (0, cy), (w, cy), (0, 255, 0), 1)
            cv2.putText(img, "NW", (10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(img, "NE", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(img, "SW", (10, cy + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(img, "SE", (cx + 10, cy + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            self._last_vis_img = img.copy()
            cv2.imshow("Strawberry Detection", img)
            key = cv2.waitKey(1) & 0xFF

            # ── 키 입력 ───────────────────────────────────────────────────────
            if ord("1") <= key <= ord("9"):
                idx = key - ord("1")
                if idx < len(self.candidates):
                    self.selected_idx = idx
                    p = self.candidates[idx][0].pose.position
                    self.locked_pos = [p.x, p.y, p.z]
                    self.locked_tid = self._candidate_tids[idx] if idx < len(self._candidate_tids) else None
                    self.get_logger().info(
                        f"LOCK [{idx+1}] tid={self.locked_tid} X:{p.x:.3f} Y:{p.y:.3f} Z:{p.z:.3f}")

            elif key == ord("u"):
                self.locked_pos = None
                self.locked_tid = None
                self.get_logger().info("Unlock")

            elif key == ord("s"):
                if self.locked_tid is not None and self.locked_pos is not None:
                    # lock된 위치를 동결값 그대로 전송 (YOLO 노이즈 영향 없음)
                    target = make_pick_msg(self.get_clock(), self.locked_pos)
                    self.pick_pub.publish(target)
                    self._log_pick_attempt("manual_locked", target, locked_idx)
                    self.get_logger().info(
                        f"Pick 전송 (LOCKED) → X:{self.locked_pos[0]:.3f} "
                        f"Y:{self.locked_pos[1]:.3f} Z:{self.locked_pos[2]:.3f}")
                elif self.candidates:
                    target = self.candidates[self.selected_idx][0]
                    target.header.stamp = self.get_clock().now().to_msg()
                    self.pick_pub.publish(target)
                    self._log_pick_attempt("manual_selected", target, self.selected_idx)
                    p = target.pose.position
                    self.get_logger().info(
                        f"Pick 전송 [{self.selected_idx+1}] → X:{p.x:.3f} Y:{p.y:.3f} Z:{p.z:.3f}")

            elif key == ord("a"):
                self.auto_mode = not self.auto_mode
                if self.auto_mode:
                    self._auto_blocked_tids.clear()
                self.get_logger().info(f"Auto mode: {'ON — pick 완료 후 다음 딸기 자동 선택' if self.auto_mode else 'OFF'}")

            elif key == ord("x"):
                tid = None
                if self.locked_tid is not None:
                    tid = self.locked_tid
                elif self.candidates and self.selected_idx < len(self._candidate_tids):
                    tid = self._candidate_tids[self.selected_idx]
                if tid is not None:
                    self._demo_blocked_tids.add(int(tid))
                    self._auto_blocked_tids.add(int(tid))
                    if self.locked_tid == tid:
                        self.locked_pos = None
                        self.locked_tid = None
                    self.get_logger().warn(f"Demo block tid={tid} — 후보에서 제외")
                else:
                    self.get_logger().warn("Demo block 실패: 선택된 track id 없음")

            elif key == ord("y"):
                self._log_manual_label("success")

            elif key == ord("n"):
                self._log_manual_label("fail")

            elif key == ord("d"):
                self._log_manual_label("dropped")

            elif key == ord("m"):
                self._log_manual_label("missed")

            elif key == ord("h"):
                if not self._homing:
                    self._homing = True
                    self.get_logger().info("Home으로 이동 중...")
                    threading.Thread(target=self._go_home, daemon=True).start()
                else:
                    self.get_logger().warn("이미 homing 중")

            elif key == ord("q"):
                raise SystemExit

            # ── 자동 연속 수확 ────────────────────────────────────────────────
            if self.pending_auto_pick and self.auto_mode:
                self.pending_auto_pick = False
                next_idx = None
                for i in range(len(self.candidates)):
                    tid = self._candidate_tids[i] if i < len(self._candidate_tids) else None
                    if tid is None or int(tid) not in self._auto_blocked_tids:
                        next_idx = i
                        break

                if next_idx is not None:
                    p = self.candidates[next_idx][0].pose.position
                    self.locked_pos = [p.x, p.y, p.z]
                    self.locked_tid = (
                        self._candidate_tids[next_idx]
                        if next_idx < len(self._candidate_tids) else None
                    )
                    target = make_pick_msg(self.get_clock(), self.locked_pos)
                    self.pick_pub.publish(target)
                    self._log_pick_attempt("auto", target, next_idx)
                    self.get_logger().info(
                        f"Auto-pick [{next_idx+1}] tid={self.locked_tid} → "
                        f"X:{p.x:.3f} Y:{p.y:.3f} Z:{p.z:.3f}")
                else:
                    self.get_logger().info("Auto mode: 더 이상 시도할 딸기 없음 — Auto OFF")
                    self.auto_mode = False

        except SystemExit:
            raise
        except Exception as e:
            self.get_logger().error(f"Loop Error: {e}")
            import traceback; traceback.print_exc()


def main():
    rclpy.init()
    node = StrawberryYoloNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
