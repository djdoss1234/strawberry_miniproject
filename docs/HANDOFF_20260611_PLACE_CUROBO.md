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

---

## 3. 예상 문제 및 대처

### 3-1. cuRobo plan 실패 시
로그에서 `MARKER_PLACE_RELEASE_DESCEND: cuRobo Cartesian plan failed` 확인.

원인 후보:
- Release 위치가 cuRobo world model에서 collision으로 판단
- Orientation이 cuRobo IK가 못 푸는 형태

대처:
```python
# curobo_planner_node.py 상단에서 확인
WALL_QUAT_WXYZ = [0.497, -0.497, 0.503, 0.503]  # 수확용 쿼터니언
```
place 쿼터니언(ZYZ → wxyz)이 수확 쿼터니언과 많이 다른지 확인.
큰 차이면 tray 스캔 시 orientation이 잘못 저장됐을 수 있음.

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

1. **Place cuRobo preview 실기 검증** (`execute_marker_place_release:=false`)
2. ABOVE 위치와 clearance 확인 후 release 활성화 검증
3. VERIFY_GRASP 서비스 연결 (gripper read_state)
4. KPI 수집 (`label_harvest_attempt.py`) — pick+place 안정 후
5. 비전 타겟 일관성 (x=-345 vs -401mm 편차)
6. NW/NE 셀 파라미터 조정
