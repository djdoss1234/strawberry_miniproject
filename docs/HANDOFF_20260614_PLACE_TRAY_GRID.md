# HANDOFF — Place Tray Grid Validation

Date: 2026-06-14 KST (세션 종료)

---

## 1. 현재 코드 상태

**최신 커밋**: `039088e feat: calibrate marker tray grid with taught pitch`

빌드 상태: 정상. 실기 전 재빌드 불필요.

```bash
source ~/doosan_ws/install/setup.bash
```

---

## 2. 완료된 것

### 2026-06-12 — Slot0 pick+place 실기 성공 (1회 육안 확인)

runtime log: `logs/runtime/2026-06-12/curobo_planner_node_20260612T184149-ec80505c.jsonl`

관찰된 시퀀스:
```
SW pick → cuRobo pre-approach → TOOL+Z 직선 → close → BASE-Z detach →
TOOL-Z retreat → cuRobo Slot0 Above → BASE-Z 120mm 하강 → release → 상승 → HOLD
```

자동 검증: `PLACE_SEQUENCE_COMPLETE_UNVERIFIED` (read_state 불가 → GRASP_UNVERIFIED)

### 2026-06-14 — Pick 정책 정리 (코드+빌드 완료, 실기 확인됨)

- KP1 근처 파지: `KP0→KP1 80%` 지점, fusion BASE-Z trim=0, `GRASP_Z_BIAS=0mm`
- 열린 그리퍼 하강: `CRANE_Z_OFFSET_M=30mm` → KP1보다 30mm 위에서 수평 진입 후 BASE-Z 30mm 하강 → close
- pre-approach IK seed 감소 (48→24), 수평 0° 후보 우선
- close 후 안정화 대기 1.5s→0.3s, 파지 검증 timeout 20s→5s

runtime log 확인: `logs/runtime/2026-06-14/curobo_planner_node_20260614T151240-6aa797eb.jsonl`
```
KP0=[−124.45,742.34,508.94]mm, KP1=[−125.43,742.12,527.73]mm
선택 offset=15.05mm (80%), fusion 목표=[−125.24,742.17,523.97]mm
pre-approach 후보 reject=0건 (수평 0° 즉시 성공)
Slot2 Above cuRobo plan: 1.90s, success
TAUGHT_TRAY_SLOT2_PLACE_PREVIEW_HOLD
```

### 2026-06-14 — Slot1/Slot3 티칭으로 15-slot 격자 생성 (코드 완료)

실측 pitch:

| 기준 | TCP BASE [x,y,z,rx,ry,rz] mm/deg |
|---|---|
| Slot0 | [519.95, 52.39, 65.58, 8.43, 90.35, -87.20] |
| Slot1 | [460.24, 55.83, 66.47, 8.43, 90.36, 87.20] |
| Slot3 | [511.91, 1.83, 63.12, 8.43, 90.37, -87.20] |

```
Slot0→Slot1: [-59.71, +3.44, +0.89]mm → 세로 pitch 59.8mm
Slot0→Slot3: [-8.04, -50.56, -2.46]mm → 가로 pitch 51.3mm
```

tray 배치:
```
Slot0  Slot3  Slot6  Slot9  Slot12
Slot1  Slot4  Slot7  Slot10 Slot13
Slot2  Slot5  Slot8  Slot11 Slot14
```

### 2026-06-14 — Marker+pitch 격자 구현 (코드 완료, 실기 미검증)

`use_taught_slot0_place_reference:=false` 경로:
- 최신 tray_cells JSON에서 마커 위치/방향을 읽음
- slot0/1/3 contact를 기준으로 마커 방향 추출
- 실측 pitch(59.8mm, 51.3mm) 적용 → 목표 contact 보정
- cuRobo FK Slot0 orientation 유지

`use_taught_slot0_place_reference:=true` 경로 (2026-06-14 Slot2 preview에서 테스트됨):
- 고정 Slot0 FK + grid offset으로 직접 Above 생성
- tray JSON 불필요 (tray 이동 시 재티칭 필요)

---

## 3. 즉시 해야 할 것

### Step 1: tray 재스캔 필수

최신 JSON 파일: `tray_cells_20260612_183939.json` (2026-06-12 18:39, **약 45시간 경과**)
max_age=3600s를 크게 초과. 재스캔 없이 marker-based preview 실행 불가.

```bash
# 터미널 1: tray 재스캔
cd ~/Downloads/share_tray && python3 run_tray_localization.py
# → 스캔 완료 후 수동으로 홈 복귀
```

### Step 2: Slot2 marker-based Above preview (release OFF)

```bash
# 터미널 2: planner
source ~/doosan_ws/install/setup.bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p use_taught_slot0_place_reference:=false \
  -p initial_place_slot_index:=2 \
  -p execute_marker_place_release:=false \
  -p measured_tcp_plan_only:=false \
  -p allow_unverified_grasp_place:=true
```

```bash
# 터미널 1 (재스캔 후): harvest 시퀀스 시작
source ~/doosan_ws/install/setup.bash
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true \
  enable_fusion_detection:=true \
  enable_pick_integration:=true \
  target_cell:=root/sw
```

**기대 로그**:
```
MARKER_TRAY_GRID slot=2 ...
5 marker place slot=2 ...
MARKER_PLACE_PREVIEW_HOLD: above reached; release disabled.
```

**육안 확인**: Slot2 Above 위치가 실제 Slot2 계란 홈 위(약 90~150mm)인지 확인.
문제 없으면 Step 3으로.

### Step 3: Slot2 marker-based release 활성화

Preview OK 확인 후에만:

```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p use_taught_slot0_place_reference:=false \
  -p initial_place_slot_index:=2 \
  -p execute_marker_place_release:=true \
  -p measured_tcp_plan_only:=false \
  -p allow_unverified_grasp_place:=true
```

주의: `allow_generated_tray_slot_release` 기본값은 `false` → Slot0 이외 슬롯에서
BASE-Z 하강/release를 차단. Slot2 실제 release를 허용하려면 추가 파라미터 필요:

```bash
  -p allow_generated_tray_slot_release:=true
```

이 플래그는 계산 기반 슬롯을 검증하기 전에는 사용하지 않는다.

---

## 4. 안전 게이트 (변경 금지)

| 파라미터 | 현재 기본값 | 의미 |
|---|---|---|
| `execute_marker_place_release` | `false` | 명시 승인 없이 release 차단 |
| `use_taught_slot0_place_reference` | `false` | 마커 기반 경로 사용 |
| `allow_generated_tray_slot_release` | `false` | 계산 생성 슬롯 release 차단 |
| `hold_after_taught_slot0_place` | `true` | Slot0 완료 후 자동 진행 차단 |
| `allow_unverified_grasp_place` | `false` | 파지 미확인 시 place 차단 (실기 시 true 필요) |

---

## 5. 미완료 (우선순위 순)

1. **tray 재스캔 → Slot2 marker-based Above preview 실기**
2. Slot2 Above 위치 육안 확인 후 `allow_generated_tray_slot_release:=true` 검증
3. Slot12, Slot14 모서리 Above preview 검증
4. 모서리 확인 후 `hold_after_taught_slot0_place:=false` 연속 place 허용
5. gripper read_state 안정화 (현재 `GRASP_UNVERIFIED` 고착)
6. 전체 15-slot 채움 후 KPI 수집 (`label_harvest_attempt.py`)
7. NW/NE 셀 파라미터 조정

---

## 6. 주요 파라미터 (현재 코드 기준)

```python
GRASP_Z_BIAS             = 0.000     # KP1 80% 지점 타겟, 추가 보정 없음
PRE_APPROACH_OFFSET      = 0.060     # 60mm pre-approach
CRANE_Z_OFFSET_M         = 0.030     # 열린 그리퍼 30mm 위에서 진입 후 하강
DETACH_PULL_DOWN_MM      = 40.0      # BASE -Z 분리 거리
TAUGHT_SLOT0_ABOVE_CLEARANCE_M = 0.120   # Slot0 위 수직 여유
TAUGHT_SLOT0_VERTICAL_VEL_MM_S = 40.0   # 수직 하강/상승 속도
LEFTMOST_WALL_SAFETY_MARGIN_M  = -0.030  # 좌측 벽 여유 -30mm
```

---

## 7. 절대 수정/커밋 금지

- `scripts/측정.py` — 사용자 파일

---

## 8. 롤백

taught 기반 경로로 롤백:
```bash
-p use_taught_slot0_place_reference:=true
```

legacy pick으로 롤백:
```bash
-p tool_model_profile:=legacy_160mm
```
