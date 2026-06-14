# HANDOFF — Place Tray Grid (최신: 2026-06-14 세션2)

---

## 1. 최신 커밋 이력

```
3c7ea3c feat: enforce operation speed 100% before place spline  ← 오늘 추가
709edf8 docs: add 2026-06-14 handoff for marker tray grid validation
039088e feat: calibrate marker tray grid with taught pitch
4fa7c86 safety: block release to unverified generated tray slots
```

빌드 필요:
```bash
cd ~/doosan_ws
colcon build --packages-select e0509_gripper_description
source install/setup.bash
```

---

## 2. 오늘(2026-06-14 세션2) 완료된 것

### Slot0 pick+place 성공 (재확인)

- `change_operation_speed(100)` 추가로 이전 세션의 Slot0 spline 실패 수정
- 로그:
  ```
  Operation speed set to 100%
  Plan OK 1645ms → end_J=[4.4, 31.0, 123.9, 175.6, 64.6, 94.7]°
  Spline 12pts exec=1.26s
  TAUGHT_SLOT0_RELEASE_DESCEND BASE -Z 120mm
  TAUGHT_TRAY_SLOT0_PLACE_RELEASE
  TAUGHT_TRAY_SLOT0_PLACE_COMPLETE_HOLD
  ```
- place 시퀀스 완전 동작 확인

### MoveSplineJoint async 동작 확인 (중요)

`execute_spline()`에서 `req.sync_type=0`은 **ASYNC** (즉시 리턴)임이 확인됨:
- Slot0 spline 전송 후 `future.done()` = 0.05초 만에 True
- Slot2 spline도 동일 0.05초
- **Slot0의 8.77초 gap**은 spline future 대기가 아니라, 이후 `execute_base_z_relative()`의 `wait_for_service`가 백그라운드 spline 모션 완료를 기다렸기 때문

> MoveLine(`sync_type=0`) = blocking  
> MoveSplineJoint(`sync_type=0`) = async (모션 accept 후 즉시 리턴)

### Slot2 Above spline 전송 (결과 미확인)

```
Plan OK 1669ms 53pts 0.95s | goal=[657.7, 97.3, 185.8]mm
end_J=[8.4, 22.2, 143.6, 180.0, 75.4, 92.8]°
→ TAUGHT_TRAY_SLOT2_RELEASE_BLOCKED (예상대로 — preview_hold)
```

- spline command는 정상 전송, 컨트롤러 accept됨
- 이후 코드가 preview_hold → HOLD_LATCHED를 0.05초 만에 로깅
- **사용자 관찰: "로봇이 안 움직였다"** — async라서 코드 로그 시점과 모션 완료 시점이 다름
- **다음 세션에서 직접 확인 필요**: Slot2 Above로 실제 이동했는지

---

## 3. 미확인 / 미해결 이슈

### [우선 1] Slot2 Above 실제 도달 여부 확인

내일 첫 번째로 Slot2 테스트 재시도:
```bash
source ~/doosan_ws/install/setup.bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p use_taught_slot0_place_reference:=true \
  -p initial_place_slot_index:=2 \
  -p execute_marker_place_release:=true \
  -p measured_tcp_plan_only:=false \
  -p allow_unverified_grasp_place:=true
```

기대 로그:
```
Operation speed set to 100%
Plan OK ... goal=[657.7, 97.3, 185.8]mm
Spline 12pts ...
TAUGHT_TRAY_SLOT2_RELEASE_BLOCKED  ← 예상 (release 없이 Above 홀드)
```

로봇이 계란판 Slot2 위 약 120mm에 멈추면 → **Slot2 Above 성공**

### [우선 2] Slot2 release 활성화

Above 위치 육안 확인 후에만:
```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p use_taught_slot0_place_reference:=true \
  -p initial_place_slot_index:=2 \
  -p execute_marker_place_release:=true \
  -p allow_generated_tray_slot_release:=true \
  -p measured_tcp_plan_only:=false \
  -p allow_unverified_grasp_place:=true
```

### [우선 3] 픽 실패 (꺾인 줄기)

이번 세션에서 2회 픽 실패:
- `GRIPPER_CLOSE success: position=700` (완전 닫힘 = 아무것도 안 잡음)
- 원인: "꺾인 줄기 살짝 옆으로 빗겨감"
- 이건 depth retry(GRASP_RETRY_OFFSETS)로 해결 안 됨 — lateral miss
- **단기 대응**: 덜 꺾인 딸기 타겟으로 교체, place 검증 먼저
- **장기 대응**: TOOL+Y lateral offset retry 추가 (아직 구현 없음)

---

## 4. 코드 상태 (현재 기준)

### 파라미터

```python
LEFTMOST_WALL_SAFETY_MARGIN_M  = -0.030   # -30mm (실기 확인)
GRASP_Z_BIAS                   = 0.000
PRE_APPROACH_OFFSET            = 0.060    # 60mm
CRANE_Z_OFFSET_M               = 0.030    # 30mm 위 진입 후 하강
DETACH_PULL_DOWN_MM            = 40.0
TAUGHT_SLOT0_ABOVE_CLEARANCE_M = 0.120    # Slot Above 높이
TAUGHT_SLOT0_VERTICAL_VEL_MM_S = 40.0
SPLINE_TIME_SCALE              = 1.125
SPLINE_MIN_TIME                = 0.75
MAX_SPLINE_POINTS              = 12
HOME_JOINTS_DEG                = [88.0, -80.0, 130.0, 0.0, 20.0, -90.0]
```

### 티칭 좌표 (변경 없음)

| 슬롯 | TCP [x,y,z,rx,ry,rz] mm/deg |
|------|------------------------------|
| Slot0 | [519.95, 52.39, 65.58, 8.43, 90.35, -87.20] |
| Slot1 | [460.24, 55.83, 66.47, 8.43, 90.36, 87.20] |
| Slot3 | [511.91, 1.83, 63.12, 8.43, 90.37, -87.20] |

Grid pitch:
- 세로(Slot0→1): [-59.71, +3.44, +0.89]mm
- 가로(Slot0→3): [-8.04, -50.56, -2.46]mm

### tray 배치

```
Slot0  Slot3  Slot6  Slot9  Slot12
Slot1  Slot4  Slot7  Slot10 Slot13
Slot2  Slot5  Slot8  Slot11 Slot14
```

### 안전 게이트 (기본값)

| 파라미터 | 기본값 | 의미 |
|---------|-------|------|
| `execute_marker_place_release` | `false` | release 명시 승인 필요 |
| `allow_generated_tray_slot_release` | `false` | 계산 생성 슬롯 release 차단 |
| `hold_after_taught_slot0_place` | `true` | Slot0 완료 후 자동 진행 차단 |
| `allow_unverified_grasp_place` | `false` | 파지 미확인 시 place 차단 |
| `use_taught_slot0_place_reference` | `false` | 마커 기반 경로 (현재 broken) |

### 마커 기반 tray localization (현재 사용 불가)

카메라가 계란판 반대 방향(-Y)을 보고 있어서 모든 최신 tray JSON이 틀린 위치를 가리킴.
`use_taught_slot0_place_reference:=true` 고정으로 우회 중.

---

## 5. 실행 명령 정리

### 기본 (Slot0 전체 pick+place)
```bash
source ~/doosan_ws/install/setup.bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p use_taught_slot0_place_reference:=true \
  -p initial_place_slot_index:=0 \
  -p execute_marker_place_release:=true \
  -p measured_tcp_plan_only:=false \
  -p allow_unverified_grasp_place:=true
```

### Slot2 Above preview (release 없음, 기본)
```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p use_taught_slot0_place_reference:=true \
  -p initial_place_slot_index:=2 \
  -p execute_marker_place_release:=true \
  -p measured_tcp_plan_only:=false \
  -p allow_unverified_grasp_place:=true
```

### Slot2 release 활성화 (Above 확인 후에만)
```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p use_taught_slot0_place_reference:=true \
  -p initial_place_slot_index:=2 \
  -p execute_marker_place_release:=true \
  -p allow_generated_tray_slot_release:=true \
  -p measured_tcp_plan_only:=false \
  -p allow_unverified_grasp_place:=true
```

---

## 6. 절대 수정/커밋 금지

- `scripts/측정.py` — 사용자 파일

---

## 7. 롤백

```bash
-p use_taught_slot0_place_reference:=false   # 마커 기반 (현재 broken)
-p tool_model_profile:=legacy_160mm          # legacy 픽 모델
```
