# Codex 인계 — Place cuRobo 검증

Date: 2026-06-11 KST (저녁 세션 종료)

---

## 1. 즉시 해야 할 것

**방금 수정된 place 코드를 먼저 preview 실기 테스트해야 한다.**

```bash
# 터미널 1: tray 재스캔 (max_age=3600s, 1시간 초과 시 필수)
cd ~/Downloads/share_tray && python3 run_tray_localization.py
# → 스캔 완료 후 수동으로 홈 복귀

# 터미널 2: curobo planner (place 활성화)
source ~/doosan_ws/install/setup.bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p execute_marker_place_release:=false \
  -p measured_tcp_plan_only:=false \
  -p allow_unverified_grasp_place:=true

# 터미널 1 (tray 재스캔 후): launch 파일
source ~/doosan_ws/install/setup.bash
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true \
  enable_fusion_detection:=true \
  enable_pick_integration:=true \
  target_cell:=root/sw
```

---

`execute_marker_place_release:=false`로 ABOVE 위치와 clearance를 먼저 확인한 뒤,
문제가 없을 때만 `true`로 바꾸어 release를 검증한다.

## 2. 이번 세션에서 해결한 것

### 2-1. Pick 파라미터 (이전 세션 이미 완료, 안정)
| 파라미터 | 값 | 설명 |
|---|---|---|
| `MEASURED_TCP_FINAL_STANDOFF_M` | -0.120 | 기준 180mm 진입 |
| `Y_DETECTION_BIAS_M` | 0.000 | raw_Y 그대로 적응형 계산 |
| `GRASP_Z_BIAS` | 0.020 | KP0 +20mm 파지 목표 |
| `PRE_APPROACH_OFFSET` | 0.060 | 6cm pre-approach |
| `CRANE_Z_OFFSET_M` | 0.000 | 크레인 비활성 |
| `DETACH_PULL_DOWN_MM` | 40.0 | BASE -Z 당기기 |

**적응형 진입**: `final_approach_distance = max(180mm, min(raw_Y - 612mm, 260mm))`

### 2-2. Place 문제 분석 및 코드 수정 (이번 세션)

**문제**: BASE ABS `RELEASE_DESCEND` (z=627mm)에서 arm 재구성 발생
- TRAY_VIEW_JOINTS (J3=112°) → release 위치 (J3=22°) 로 플립
- Doosan MoveLine이 37초 소요, 딸기 35cm 공중 낙하

**원인**: TRAY_VIEW_JOINTS orientation abc=[-3.4, 140.7, -92.6]°에서
z=627mm BASE ABS로 하강 시 kinematic 충돌 → 다른 IK solution 선택

**수정 내용** (`scripts/curobo_planner_node.py`):
1. `scipy.spatial.transform.Rotation` import 추가
2. `_doosan_zyz_to_wxyz(rx, ry, rz)` helper 추가 (ZYZ → quaternion)
3. `_execute_marker_place_after_retreat` 내 BASE ABS 이동 3개 → cuRobo 대체:
   - `execute_base_line(above)` → `self.plan(tray_view_joints, above_pos_m, above_quat)` + `execute_spline`
   - `execute_base_line(release)` → `self.plan(above_joints, release_pos_m, release_quat)` + `execute_spline`
   - release 후 `self.plan(release_joints, above_pos_m, above_quat)`로 ABOVE 복귀
   - ABOVE에서 `plan_to_fixed_joints_pose(..., TRAY_VIEW_JOINTS_DEG)`로 복귀
4. 추가 안전 보강:
   - ABOVE cuRobo plan 실패 시 tray-view를 ABOVE로 간주하지 않고 fail-closed
   - release 위치에서 tray-view로 바로 관절 이동하지 않고 반드시 ABOVE를 경유
   - tray-view 복귀 시 swing check를 생략하지 않음

**실기 미검증** — 다음 세션 첫 번째 작업

### 2-3. Tray-view 정지 원인 및 measured place 좌표 수정

run `curobo_planner_node_20260611T191813-479a10b2.jsonl`에서 pick과 tray-view
이동은 성공했지만 ABOVE `(489.6,-325.5,720.7)mm`가 `IK_FAIL`로 거부되었다.
이 정지는 fail-closed가 정상 동작한 결과다.

원인은 tray JSON의 `position_tcp_mm`가 기존 Robotis TCP에서 연장 파츠 120mm를
보정한 좌표인데, measured `grasp_tcp_link`에 다시 적용되어 tool 길이 보정이
중복된 것이다.

수정 후 measured profile은:

- `position_contact_mm` 사용
- 파츠 끝보다 10mm 뒤의 실제 파지 중심으로 변환
- 최신 slot0 release 약 `(559.2,-329.6,535.6)mm`
- 최신 slot0 ABOVE 약 `(559.2,-329.6,635.6)mm`

legacy profile만 기존 `position_tcp_mm`를 사용한다.

### 2-4. Corrected 위치에서도 IKFAIL: tray-view FK orientation 사용

run `curobo_planner_node_20260611T193642-32d972cf.jsonl`에서는 corrected ABOVE
`(559.2,-329.6,635.6)mm`가 적용됐지만 tray JSON orientation에서 다시 IKFAIL이
발생했다.

따라서 place orientation source를 변경했다.

- tray JSON orientation: 좌표 계산 및 비교 기록용
- 실제 cuRobo place orientation: 이미 도달한 tray-view joints의 cuRobo FK
- ABOVE, RELEASE, ABOVE retreat 동안 같은 orientation 유지
- JSON과 FK orientation 차이는 runtime JSONL에 각도로 기록

다음 테스트도 `execute_marker_place_release:=false`로 ABOVE preview만 수행한다.

### 2-5. Tray-view FK도 IKFAIL: top-down orientation 후보 추가

run `curobo_planner_node_20260611T194304-59f31711.jsonl`에서 tray-view FK
orientation도 corrected ABOVE에서 IKFAIL이었다.

현재 수정:

- tray-view FK orientation 1개
- TOOL +Z가 아래를 향하는 top-down yaw `0, ±45, ±90, 180deg`
- 총 7개 후보 중 cuRobo가 도달 가능한 첫 경로 선택
- 각 후보 orientation에 맞춰 contact point에서 grasp center 10mm 보정을 재계산
- 선택 결과를 runtime JSONL에 기록

다음 테스트는 반드시 release를 끈 상태로 ABOVE preview를 확인한다.

### 2-6. 모든 자세 후보 IKFAIL: ABOVE clearance 작업반경 수정

run `curobo_planner_node_20260611T195103-6e5b1249.jsonl`에서 7개 orientation이
모두 IKFAIL이었다.

- release 목표 거리: 약 `0.823m`
- ABOVE 100mm 목표 거리: 약 `0.893m`
- 원인: 계란판이 너무 가까운 것이 아니라 ABOVE가 작업반경 경계까지 멀어진 것

현재 수정:

- ABOVE clearance `100, 70, 50, 30mm` 순차 탐색
- 각 clearance마다 orientation 후보 탐색
- 파츠 끝 contact가 계란판 면에서 이미 60mm 위이므로 추가 30mm도 preview 시
  총 약 90mm clearance 확보

다음 실행도 release를 끈 preview다.

### 2-7. 완전 top-down은 260mm tool 때문에 flange가 작업반경 밖

run `curobo_planner_node_20260611T195746-cff71f1b.jsonl`에서 낮은 ABOVE까지
모두 IKFAIL이었다.

원인:

- cuRobo target은 260mm 연장된 grasp TCP다.
- 완전 top-down이면 목표 TCP보다 flange가 260mm 위에 있어야 한다.
- slot0 기준 top-down implied flange 거리 약 `1.05m`로 도달 불가
- 사선 하향이면 implied flange 거리가 약 `0.75~0.85m`로 줄어듦

현재 수정:

- 계란판 방향으로 기울어진 사선 하향 후보 추가
- 하향 성분 `0.25/0.50/0.75`, roll `0/±90/180deg`
- 로그에 `tcp_r`, `flange_r` 출력

다음 테스트도 release-off preview로 수행한다.

### 2-8. 사선 하향 Plan OK지만 실기 preview 실패

run:

```text
logs/runtime/2026-06-11/
curobo_planner_node_20260611T200428-57f83553.jsonl
```

결과:

- `inclined_down_0.25_roll_+0`, clearance `100mm` 후보가 최초로 Plan OK
- end joints:
  `[-29.2,-21.4,78.9,178.8,-46.6,181.1]deg`
- release-off preview hold는 정상 동작
- 그러나 관절이 크게 회전하고 계란판 위가 아닌 공중의 부적절한 자세로 이동
- 실기 기준 실패이며 place 성공으로 기록하면 안 됨

중요 결론:

- 자동 orientation 후보를 더 늘리는 방식은 중단한다.
- IK 성공만으로 place 경로를 선택하면 안 된다.
- 다음 작업은 계란판 위의 적절한 place 전용 기준 관절 자세를 티칭/검증하고,
  기준 자세 대비 joint delta와 path quality로 후보를 제한하는 것이다.
- 검증 전까지 `execute_marker_place_release:=false` 유지.

---

## 3. 예상 문제 및 대처

### 3-1. 자동 place preview 재실행 금지

현재 자동 orientation sampling은 IK가 성공해도 실기상 부적절한 관절 branch와
공중 자세를 선택했다. place 전용 기준 관절 자세를 확보하기 전에는 자동 place
preview 및 release를 반복 실행하지 않는다.

### 3-2. 위치는 맞는데 높이가 여전히 틀릴 경우
tray_cells JSON의 `position_tcp_mm.z`가 실제 egg hole z와 다를 수 있음.
→ tray 재스캔 후 JSON의 slot0 z값이 실제 계란판 홈 높이와 맞는지 육안 확인.

slot0 z=627~634mm 범위여야 함 (로봇 base에서 계란판 홈까지 높이).
실제로 다르면 `run_tray_localization.py` 설정 확인 필요.

### 3-3. `allow_unverified_grasp_place:=true` 주의
파지 실패해도 place 진행함. 반드시 사람이 파지 성공 여부 육안 확인 필요.
GRASP_UNVERIFIED = read_state 서비스 없어서 자동 판정 불가.

---

## 4. 슬롯 순서 및 tray JSON

```
슬롯 배치:
        col0   col1   col2
row0: [ 00 ] [ 01 ] [ 02 ]  ← TRAY_VIEW_JOINTS 아래 (slot0 ≈ scan pose)
row1: [ 03 ] [ 04 ] [ 05 ]
row2: [ 06 ] [ 07 ] [ 08 ]
row3: [ 09 ] [ 10 ] [ 11 ]
row4: [ 12 ] [ 13 ] [ 14 ]
```

- `_marker_place_slot_idx`: 노드 시작 시 0, 성공마다 +1 (노드 재시작 시 0으로 초기화)
- tray JSON max_age: 3600초. 1시간 초과 시 재스캔 필요
- 최신 JSON: `~/Downloads/share_tray/output/tray_cells_YYYYMMDD_HHMMSS.json`

---

## 5. 주요 파일

| 파일 | 설명 |
|---|---|
| `scripts/curobo_planner_node.py` | 핵심 플래너 (이번 세션 수정됨) |
| `docs/harvest_motion_session_20260611.md` | 전체 세션 기록 |
| `docs/HANDOFF_20260611_MEASURED_TCP.md` | measured TCP 전환 배경 |
| `config/curobo/e0509_gripper_measured_tcp.yml` | cuRobo measured TCP 프로필 |
| `~/Downloads/share_tray/run_tray_localization.py` | tray 스캔 스크립트 (읽기 전용) |

**절대 수정 금지**: `scripts/측정.py`

---

## 6. 안전 규칙

- 실기 실행 전 E-stop 준비
- `allow_unverified_grasp_place:=true` 상태이므로 빈 그리퍼로 place 시도 가능 → 사람 관찰 필수
- tray scan 후 반드시 수동으로 홈 복귀 확인 (`run_tray_localization.py` 는 자동 홈 복귀 안 함)
- Place Cartesian plan과 release 후 tray-view 복귀는 swing check를 통과해야 실행됨

---

## 7. 미완료 작업 (우선순위 순)

1. **계란판 place 전용 기준 관절 자세 저속 티칭/검증**
2. 기준 자세 대비 joint delta/path quality 제한 구현
3. marker 이동량을 기준 자세와 결합한 preview 검증
4. 검증 후에만 release 활성화
5. VERIFY_GRASP 서비스 연결 (gripper read_state)
6. KPI 수집 (`label_harvest_attempt.py`) — pick+place 안정 후
7. 비전 타겟 일관성 (x=-345 vs -401mm 편차)
8. NW/NE 셀 파라미터 조정

---

## 8. 2026-06-12 slot0 수동 기준 자세 확보

자동 place orientation sampling의 부적절한 관절 branch를 대체할 place 전용
기준 후보를 사용자가 직접 확인했다.

```yaml
joints_deg: [5.34, 3.05, 124.87, 179.46, -3.46, 93.51]
posx_mm_deg: [441.65, 41.20, 233.76, 5.30, 131.37, -87.05]
```

아직 `above` 또는 `release`로 분류하지 않은 기준 후보이며 자동 release에는
연결하지 않는다. 다음 작업은 빈 그리퍼 저속 검증 후 이 자세를 preferred joint
branch로 사용하고, marker 목표의 위치 변화만 제한적으로 반영하는 것이다.

2026-06-12 변경: 고정 slot0 자세를 사용할 때는 불필요한 overview/tray-view
경유를 제거했다. 현재 경로는 pick retreat 자세에서 slot0 기준 자세로 직접
cuRobo joint-space planning 후 이동하며, release 후에도 tray-view를 경유하지 않는다.

### 2026-06-12 파지 목표 정리

- 물리적 목표: KP0 바로 위 `10~20mm`의 가는 줄기 부분
- 꺾인 줄기: 전체 `KP0 -> KP2` 직선이 아니라 국소 `KP0 -> KP1` 방향 사용
- fusion의 base-Z trim `+10mm`와 planner의 `GRASP_Z_BIAS +20mm`가 중복 적용될
  가능성이 확인됨
- 다음 실기는 place 비활성 상태에서 파지 위치를 먼저 확인하고, 목표가 높으면
  planner 측 `GRASP_Z_BIAS`를 우선 제거/파라미터화한다
