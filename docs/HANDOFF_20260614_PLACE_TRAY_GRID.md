# HANDOFF — Place Tray Grid (최신: 2026-06-15)

---

## 1. 최신 커밋 이력

```
3c7ea3c feat: enforce operation speed 100% before place spline
709edf8 docs: add 2026-06-14 handoff for marker tray grid validation
039088e feat: calibrate marker tray grid with taught pitch
4fa7c86 safety: block release to unverified generated tray slots
```

> ⚠️ 2026-06-15 변경사항은 아직 커밋 안 됨:
> - `scripts/curobo_planner_node.py`: J3 한계, row2 파라미터, descent/ascent 분기
> - `config/curobo/e0509_gripper.urdf`: J3 limit ±2.356194 rad

빌드 필요:
```bash
cd ~/doosan_ws
colcon build --packages-select e0509_gripper_description
source install/setup.bash
```

---

## 2. 슬롯 레이아웃

```
Slot0  Slot3  Slot6  Slot9  Slot12   (row0 — BASE -Z 방식)
Slot1  Slot4  Slot7  Slot10 Slot13   (row1 — BASE -Z 방식)
Slot2  Slot5  Slot8  Slot11 Slot14   (row2 — cuRobo plan 필요)
```

Grid pitch (측정값):
- 세로(row 방향, Slot0→1): [-59.71, +3.44, +0.89]mm
- 가로(col 방향, Slot0→3): [-8.04, -50.56, -2.46]mm

티칭 TCP 좌표 (mm, deg):
```
Slot0: [519.95,  52.39, 65.58, 8.43, 90.35, -87.20]
Slot1: [460.24,  55.83, 66.47, 8.43, 90.36, -87.20]
Slot3: [511.91,   1.83, 63.12, 8.43, 90.37, -87.20]
```

---

## 3. 2026-06-15 완료된 것

### 3-1. J3 실측 한계 ±135° 수정

**증상**: Slot2 descent에서 J3=137.55° 도달 → Doosan이 silent reject (success=True지만 0.05s 즉시 리턴, 모션 없음)

**수정 1** — `config/curobo/e0509_gripper.urdf`:
```xml
<joint name="joint_3" type="revolute">
  <limit effort="194" lower="-2.356194" upper="2.356194" velocity="3.1416" />
</joint>
```

**수정 2** — `scripts/curobo_planner_node.py` line ~152:
```python
OPERATIONAL_JOINT_LIMITS_DEG = [
    (-225.0, 225.0),
    (-95.0,   95.0),
    (-135.0, 135.0),   # J3: 실측 확인 ±135° (2026-06-15, 기존 ±155° 잘못됨)
    (-280.0, 280.0),
    (-135.0, 135.0),
    (-360.0, 360.0),
]
```

### 3-2. row2 place 문제 해결

**문제**: Slot2+ release 위치에서 J3>135° 없이는 reachable 위치가 없음 → BASE -Z 불가

**시도 흐름**:
1. cuRobo plan (tilt 0°): IK_FAIL — J3≤135° 제약으로 해 없음
2. pitch tilt 15° 시도: Above OK(J3=120.6°), release IK_FAIL
3. **pitch tilt 30° → 성공**: Above OK, release OK, place 동작

**위치 오차 보정**:
- 30° tilt 때문에 TCP 오프셋 3~4cm 발생
- 방향: Slot1→2 방향 (BASE -X 방향)
- 보정: `row2_release_correction_mm=[35,-2,0]`

### 3-3. row2 전용 파라미터 추가

`scripts/curobo_planner_node.py` line ~334:
```python
self.declare_parameter("taught_slot_above_clearance_m", TAUGHT_SLOT0_ABOVE_CLEARANCE_M)
self.declare_parameter("row2_place_pitch_tilt_deg", 15.0)
self.declare_parameter("row2_release_correction_mm", [0.0, 0.0, 0.0])
```

line ~367:
```python
self._taught_slot_above_clearance_m = float(
    self.get_parameter("taught_slot_above_clearance_m").value)
self._row2_place_pitch_tilt_deg = float(
    self.get_parameter("row2_place_pitch_tilt_deg").value)
self._row2_release_correction_mm = list(
    self.get_parameter("row2_release_correction_mm").value)
```

| 파라미터 | 기본값 | 실기 row2 값 | 설명 |
|---------|-------|------------|------|
| `taught_slot_above_clearance_m` | 0.120 | 0.300 | Above 높이. row2는 300mm 필요 |
| `row2_place_pitch_tilt_deg` | 15.0 | 30.0 | row2 그리퍼 Y축 pitch tilt |
| `row2_release_correction_mm` | [0,0,0] | [35,-2,0] | tilt 오프셋 보정 |
| `row2_max_line_deviation_mm` | 20.0 | 우선 20.0 | row2 궤적의 직선 대비 최대 측방 편차 |

### 3-4. descent/ascent 분기 구현

`_execute_taught_slot0_place_reference_after_retreat` 내 핵심 로직:

```python
is_row2 = (slot_index % 3 == 2)  # Slot2,5,8,11,14

# Tilt 적용 (row2만)
if is_row2 and self._row2_place_pitch_tilt_deg != 0.0:
    w, x, y, z = release_fk_quat
    base_rot = SciR.from_quat([x, y, z, w])
    tilt_rot = SciR.from_euler('y', self._row2_place_pitch_tilt_deg, degrees=True)
    tilted = tilt_rot * base_rot
    q = tilted.as_quat()
    release_fk_quat = [float(q[3]), float(q[0]), float(q[1]), float(q[2])]

# 위치 보정 (row2만)
if is_row2 and any(v != 0.0 for v in self._row2_release_correction_mm):
    corr_m = np.array(self._row2_release_correction_mm, dtype=float) / 1000.0
    release_pos_m = (np.array(release_pos_m, dtype=float) + corr_m).tolist()

# Descent
if is_row2:
    full_traj, full_time = self.plan(above_joints, release_pos_m, release_fk_quat, ...)
    deviation_mm = self._trajectory_line_deviation_mm(
        full_traj, above_pos_m, release_pos_m)
    # 허용 편차 이내에서만 정지 없는 단일 spline 실행
    self.execute_spline(full_traj, full_time)
else:
    # row0/1: BASE -Z 직선 하강 유지
    self.execute_base_z_relative(-clearance_m, ..., TAUGHT_SLOT0_VERTICAL_VEL_MM_S)
```

### 3-5. 슬롯별 검증 결과

| 슬롯 | 상태 | 방식 | 비고 |
|------|------|------|------|
| Slot0 | ✓ 성공 | BASE -Z, 120mm | 티칭 |
| Slot1 | ✓ 성공 | BASE -Z, 120mm | 티칭 |
| Slot3 | ✓ 성공 | BASE -Z, 120mm | 티칭 |
| Slot4 | ✓ 성공 | BASE -Z, 120mm | 생성 |
| Slot2 | △ 성공 (오차 ~3cm) | cuRobo 30° tilt, 300mm | corr=[35,-2,0] |
| Slot5 | ✗ 미완 | cuRobo 3-seg 시도 | 뚝뚝 끊기고 위치 오차 |
| 나머지 | 미검증 | — | — |

---

## 4. 미해결 이슈

### [우선 1] row2 (Slot5,8,11,14) 개선

**현상**: 3-segment trajectory 실행 시 뚝뚝 끊기고 위치 오차 있음

**근본 원인**:
1. cuRobo plan이 관절 공간 최적화 → 직선 수직 경로가 아닌 곡선 생성
2. 인접 딸기/계란판 칸에 그리퍼가 스칠 수 있음
3. 단순 N-segment 분할은 매 세그먼트 시작/끝에서 정지 → "뚝뚝" 모션

**시도했지만 실패한 것**:
- 3-hop 독립 cuRobo plan: hop3에서 J4 branch 전환 오류 (J4 277°→-79.4°, 359° 불연속)
- 3-segment 단일 plan 분할: 동작은 하나 끊기고 위치 오차

**2026-06-15 Codex 후속 변경 — 실기 미검증**:

- 설치된 cuRobo에는 `MotionGenConstraintConfig` / `ee_pos_constraint` API가 없다.
- `PoseCostMetric`은 partial pose/approach cost를 제공하지만 완전한 BASE 수직
  직선을 보장하지 않으므로 즉시 적용하지 않았다.
- row2 하강·상승의 3-segment 실행을 제거하고 단일 MoveSplineJoint 실행으로
  변경하여 구간별 정지를 제거했다.
- 실행 전 전체 궤적을 FK로 변환해 Above↔release Cartesian 선분 대비 최대
  측방 편차를 계산한다.
- 기본 허용 편차는 `row2_max_line_deviation_mm=20.0`; 초과하면 release 전에
  실행을 차단한다.

첫 실기 검증은 release 없이 계획/FK 편차 로그를 확인해야 한다.

```text
ROW2_DESCENT_LINE_CHECK max_deviation=... limit=20.0mm
```

편차가 20mm를 초과하면 collision geometry 추가 또는 Cartesian waypoint IK
방식을 검토한다.

### [우선 2] Slot6,7,9,10,12,13 검증

row0/1이므로 BASE -Z 방식 그대로 동작 예상. 순서대로 검증:
```bash
-p initial_place_slot_index:=6    # → 7 → 9 → 10 → 12 → 13
```

### [우선 3] 픽 실패 (꺾인 줄기)

- 꺾인 줄기: `GRIPPER_CLOSE position=700` (완전 닫힘) = lateral miss
- 단기: 덜 꺾인 타겟 선택, 장기: TOOL +Y lateral offset retry 로직

### [낮음] 실효 속도 ~14%

`change_operation_speed(100)` 효과 없음. 조사 필요.

### [낮음] tray localization

카메라가 계란판 반대 방향 봄 → `use_taught_slot0_place_reference:=true`로 우회 중.

---

## 5. 실행 명령

### row0/1 슬롯 (Slot0,1,3,4,6,7,9,10,12,13)
```bash
source ~/doosan_ws/install/setup.bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p use_taught_slot0_place_reference:=true \
  -p initial_place_slot_index:=X \
  -p execute_marker_place_release:=true \
  -p allow_generated_tray_slot_release:=true \
  -p measured_tcp_plan_only:=false \
  -p allow_unverified_grasp_place:=true
```

### row2 슬롯 (Slot2,5,8,11,14) — 오차 있음, 개선 필요
```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p use_taught_slot0_place_reference:=true \
  -p initial_place_slot_index:=X \
  -p execute_marker_place_release:=true \
  -p allow_generated_tray_slot_release:=true \
  -p measured_tcp_plan_only:=false \
  -p allow_unverified_grasp_place:=true \
  -p taught_slot_above_clearance_m:=0.300 \
  -p row2_place_pitch_tilt_deg:=30.0 \
  -p "row2_release_correction_mm:=[35.0,-2.0,0.0]"
```

> ⚠️ `-p "key:=value"` 형식 — `-p`와 따옴표 사이 띄어쓰기 필수. `"-p key:=value"` 하면 안 됨.

---

## 6. 코드 상수 (curobo_planner_node.py)

```python
LEFTMOST_WALL_SAFETY_MARGIN_M  = -0.030   # leftmost 진입 마진
GRASP_Z_BIAS                   = 0.030    # kp0 기준 위로 30mm (현재 3~4cm 높은 파지 문제)
PRE_APPROACH_OFFSET            = 0.060    # 사전 접근 오프셋
CRANE_Z_OFFSET_M               = 0.030    # 크레인 Z 오프셋
DETACH_PULL_DOWN_MM            = 40.0     # 줄기 분리 하강
TAUGHT_SLOT0_ABOVE_CLEARANCE_M = 0.120   # row0/1 Above 기본값
TAUGHT_SLOT0_VERTICAL_VEL_MM_S = 40.0   # 수직 이동 속도
SPLINE_TIME_SCALE              = 1.125
SPLINE_MIN_TIME                = 0.75
MAX_SPLINE_POINTS              = 12
HOME_JOINTS_DEG = [88.0, -80.0, 130.0, 0.0, 20.0, -90.0]
```

### 안전 게이트 (기본값 모두 false/true = 비활성)
| 파라미터 | 기본값 | 역할 |
|---------|-------|------|
| `execute_marker_place_release` | false | place release 실행 |
| `allow_generated_tray_slot_release` | false | 생성 슬롯 release 허용 |
| `hold_after_taught_slot0_place` | true | place 후 홀드 |
| `allow_unverified_grasp_place` | false | 미검증 grasp으로 place 허용 |
| `use_taught_slot0_place_reference` | false | 티칭 기준 사용 (tray loc 우회) |

---

## 7. 절대 수정/커밋 금지

- `scripts/측정.py` — 사용자 원본 파일, 건드리지 말 것

---

## 8. 롤백

```bash
-p use_taught_slot0_place_reference:=false   # 마커 기반 (현재 broken)
-p tool_model_profile:=legacy_160mm          # legacy 픽 모델
```
