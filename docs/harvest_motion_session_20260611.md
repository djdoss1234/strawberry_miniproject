# 수확 모션 세션 — 2026-06-11

## 세션 목표

measured TCP 260mm 프로필 실기 검증 및 파라미터 안정화.
SW 셀 단일 딸기 수확 모션 반복 성공 조건 확립.

---

## 1. 시작 상태

- `MEASURED_TCP_FINAL_STANDOFF_M = 0.030` → pre-approach 60mm + 직선 30mm = 90mm 총 진입
- `GRASP_Z_BIAS = 0.030`
- plan_only 검증 완료 (9159d74), 실기 실행으로 전환

---

## 2. 시도별 문제 및 수정

### 2-1. 그리퍼가 15cm 앞에서 닫힘

**원인**: standoff=+30mm → 총 진입 90mm → 줄기가 780~800mm 위치인데 TCP가 702mm에서 닫힘  
**수정**: `MEASURED_TCP_FINAL_STANDOFF_M = -0.120` → 총 진입 180mm → TCP 792mm 도달  
**결과**: 딸기 1개 파지 성공

### 2-2. 딸기마다 진입 거리 달라야 함

**원인**: Y-clamp가 raw 감지 Y를 672mm로 통일 → 모든 딸기의 TCP 종착점이 792mm로 고정.  
실제로는 앞으로 나온 딸기(raw_Y≈790mm)와 뒤에 있는 딸기(raw_Y≈835mm)의 실제 깊이가 다름.

**수정**: 클램핑 전 raw_Y 보존 → 딸기별 적응형 진입 거리 계산
```
detection_raw_y = float(p.y)  # 클램핑 전 원본
adaptive_dist = (detection_raw_y - Y_DETECTION_BIAS_M) - (WALL_SURFACE_Y_M - PRE_APPROACH_OFFSET)
final_approach_distance = max(baseline_180mm, min(adaptive_dist, 260mm))
```

- `Y_DETECTION_BIAS_M = 0.023` 시도 → 오히려 진입 거리를 줄여버려 실패 (808mm raw → 173mm, floor 180mm)
- `Y_DETECTION_BIAS_M = 0.000` 확정 → 808~815mm raw에서 196~203mm 진입

**결과**: 뒤에 있는 딸기까지 진입 거리 자동 조정, 다수 딸기 파지 가능

### 2-3. 맨 왼쪽 딸기(z=491mm)에서 잎 간섭

**원인**: 중앙 딸기(z=584mm)보다 93mm 낮은 위치. `GRASP_Z_BIAS=0`이면 그 높이의 잎이 수평 접근 경로를 막음.

**시도 A: 크레인 접근** (위에서 진입 후 하강)
```
cuRobo → (x, 612mm, z+80mm) → TOOL+Z 200mm → BASE -Z 80mm → gripper close
```
- `execute_base_z_relative()` 메서드 추가
- **문제**: BASE -Z 하강 시 주변 다른 줄기/잎을 쓸고 내려옴 → 간섭으로 정상 줄기도 움직임
- NW/NE 같은 밀도 높은 구역에서는 더 심해질 것 → 기각

**시도 B: GRASP_Z_BIAS 조정** (검증된 수평 접근 유지, Z 목표만 위로)
- `GRASP_Z_BIAS = 0.020` → KP0 기준 20mm 위를 파지 목표로 설정
- 잎 위에서 줄기 상단부를 파지 → BASE -Z 40mm 당기기로 분리
- **결과**: 맨 왼쪽 딸기 포함 정상 동작 확인

**크레인 코드는 보존** (`CRANE_Z_OFFSET_M = 0.000`으로 비활성화). 향후 perception 기반 장애물 등록과 함께 재활성화 검토.

### 2-4. GRASP_Z_BIAS 조정 이력

| 값 | 결과 |
|----|------|
| 0.030 | kp0보다 3~4cm 위 → 높이 과도 |
| 0.010 | 계획 단계에서 테스트 |
| 0.000 | 중앙 딸기 정확, 왼쪽 딸기 잎 간섭 |
| **0.020** | **현재 확정. 다수 딸기 파지 성공** |

---

## 3. 현재 확정 파라미터

```python
MEASURED_TCP_FINAL_STANDOFF_M = -0.120   # 기준 180mm 진입
Y_DETECTION_BIAS_M            =  0.000   # raw_Y 그대로 적응형 진입 계산
GRASP_Z_BIAS                  =  0.020   # KP0 +20mm 파지 목표
PRE_APPROACH_OFFSET           =  0.060   # 6cm pre-approach
CRANE_Z_OFFSET_M              =  0.000   # 크레인 비활성
DETACH_PULL_DOWN_MM           = 40.0     # BASE -Z 당기기
```

**적응형 진입 거리**: `final_approach_distance = max(180mm, min(raw_Y - 612mm, 260mm))`
- raw_Y=808mm → 196mm
- raw_Y=815mm → 203mm
- raw_Y=835mm → 223mm

---

## 4. 수확 시퀀스 (현재 기준)

```
scan pose
  ↓ cuRobo MoveSplineJoint
pre-approach (x, 612mm, z+20mm)    ← GRASP_Z_BIAS +20mm 반영
  ↓ TOOL +Z 196~220mm (raw_Y 기반 적응)
grasp position
  ↓ gripper close (pos 700)
  ↓ VERIFY_GRASP (서비스 미연결 → GRASP_UNVERIFIED)
  ↓ BASE -Z 40mm (detach pull)
  ↓ TOOL -Z 후퇴
scan pose 복귀
```

---

## 5. Place 시퀀스 — 2026-06-11 오후 세션

### 5-1. enable_marker_place_sequence 첫 실행

파라미터:
```
enable_marker_place_sequence:=true
execute_marker_place_release:=true
measured_tcp_plan_only:=false
allow_unverified_grasp_place:=true
```

**문제 1**: `PLACE_GATE_BLOCKED (GRASP_UNVERIFIED)` — read_state 미연결로 place 차단  
**해결**: `allow_unverified_grasp_place:=true` 추가

**문제 2**: `MARKER_PLACE_RELEASE_DESCEND` 37초 소요, 딸기 35cm 공중 낙하  
**원인 분석**:
- TRAY_VIEW_JOINTS(J3=112°)에서 BASE ABS z=627mm 하강 시 arm 재구성 발생 (J3=112°→22°)
- Doosan MoveLine이 완전 다른 IK solution으로 "플립" — 37초 소요
- ABOVE_RETREAT도 동일 joints = 실제로 100mm 올라가지 않음
- 결과: TCP는 명령 좌표에 있지만 arm 자세가 틀려 실제 gripper 위치 오차 35cm

**해결 (이번 세션 마지막 커밋)**:
- `execute_base_line` (BASE ABS) → **cuRobo Cartesian plan + MoveSplineJoint**
- RETREAT도 BASE ABS → joint-space plan to TRAY_VIEW_JOINTS
- `_doosan_zyz_to_wxyz()` helper 추가 (ZYZ Euler → quaternion)
- scipy import 추가

**아직 실기 검증 안 됨** — 빌드만 완료, 다음 세션에서 테스트 필요

### 5-3. Codex 재검토 후 place 안전 보강

Claude Code 구현을 재검토하면서 실기 전 위험 분기 두 개를 수정했다.

1. ABOVE cuRobo 계획 실패 시 기존 코드는 현재 tray-view 자세를 ABOVE로
   간주하고 release 단계로 계속 진행했다.
   - 수정: ABOVE 계획 실패 시 과실을 든 상태로 즉시 중단하는 fail-closed 적용
2. release 후 기존 코드는 tray-view 관절 자세로 바로 복귀했다.
   - 위험: 계란판 위 clearance를 확보하기 전에 joint-space 경로가 tray body를
     가로지를 수 있음
   - 수정: `release -> cuRobo ABOVE -> tray-view joint-space` 순서로 변경
   - tray-view 복귀의 swing check 생략도 제거

첫 실기 검증은 `execute_marker_place_release:=false`로 ABOVE preview만 확인한 뒤,
clearance가 확인되면 release를 활성화한다.

### 5-4. Tray-view 정지 원인: legacy TCP 좌표 중복 보정

실기 run:

```text
logs/runtime/2026-06-11/
curobo_planner_node_20260611T191813-479a10b2.jsonl
```

관찰:

- pick, detach, retreat, overview, tray-view 이동 성공
- `MARKER_PLACE_ABOVE` 목표 `(489.6,-325.5,720.7)mm`에서 `IK_FAIL`
- fail-closed가 동작하여 과실을 잡은 채 tray-view에서 안전 정지

원인:

- `share_tray`의 `position_tcp_mm`는 기존 Robotis TCP에 연장 파츠 `120mm`를
  적용하기 위해 접촉점에서 뒤로 물린 좌표다.
- 현재 cuRobo measured profile의 `grasp_tcp_link`는 이미 flange 기준 `260mm`
  물리 파지 중심이다.
- measured profile에서 legacy `position_tcp_mm`를 그대로 사용하여 연장 파츠
  보정이 중복 적용되었다.

수정:

- measured profile은 `position_contact_mm`를 기준으로 사용한다.
- 실제 파지 중심이 파츠 끝보다 약 `10mm` 뒤이므로 TOOL `+Z` 반대 방향으로
  `10mm` 이동한 좌표를 release target으로 생성한다.
- legacy profile은 기존 `position_tcp_mm`를 유지한다.

최신 slot0 기준 변화:

| 기준 | release target | ABOVE target |
| --- | --- | --- |
| 잘못된 legacy TCP 중복 적용 | `(489.6,-325.5,620.7)mm` | z=`720.7mm` |
| measured grasp center 적용 | `(559.2,-329.6,535.6)mm` | z=`635.6mm` |

다음 실행은 반드시 `execute_marker_place_release:=false`로 corrected ABOVE 위치와
clearance부터 확인한다.

### 5-5. Corrected ABOVE 위치에서도 IK_FAIL: place orientation source 수정

run:

```text
logs/runtime/2026-06-11/
curobo_planner_node_20260611T193642-32d972cf.jsonl
```

관찰:

- measured grasp center 기준 목표
  `(559.2,-329.6,635.6)mm`가 정상 적용됨
- 같은 tray JSON의 `task_orientation_deg`를 cuRobo quaternion으로 변환한
  orientation에서 여전히 `IK_FAIL`

판단:

- 위치 중복 보정 문제는 해결되었고, 남은 문제는 Doosan controller TCP 자세와
  cuRobo `grasp_tcp_link` 자세 convention/model 차이다.
- tray-view 관절 자세는 실제 로봇과 cuRobo 모두 이미 도달 가능한 검증 자세다.
- place에서는 tray-view의 cuRobo FK orientation을 유지하고 slot 위치만 변경하는
  것이 가장 보수적이다.

수정:

- tray-view 도달 후 cuRobo FK로 `grasp_tcp_link` quaternion을 계산한다.
- ABOVE, RELEASE, ABOVE retreat 모두 이 orientation을 유지한다.
- JSON orientation과의 각도 차이를
  `marker_place_orientation_selected.angular_delta_deg`로 기록한다.

다음 실행도 release를 끈 preview로 corrected ABOVE 계획 성공 여부부터 확인한다.

### 5-2. 슬롯 레이아웃 확인

```
        col0   col1   col2
row0: [ 00 ] [ 01 ] [ 02 ]   ← TRAY_VIEW_JOINTS TCP 바로 아래
row1: [ 03 ] [ 04 ] [ 05 ]
row2: [ 06 ] [ 07 ] [ 08 ]
row3: [ 09 ] [ 10 ] [ 11 ]
row4: [ 12 ] [ 13 ] [ 14 ]
```

slot0부터 순서대로 채우기로 확정 (`_marker_place_slot_idx` 성공마다 +1 자동 증가).

---

## 6. 미해결 / 다음 세션 작업

### ★ 최우선 (다음 세션 첫 번째)
- [ ] **Place cuRobo 플랜 실기 검증** — 방금 커밋된 코드로 테스트
  - RELEASE cuRobo plan 성공 여부 확인 (IK 풀리는지)
  - arm 재구성 없이 slot 위치에 정확히 내려가는지 육안 확인
  - 로그에서 `MARKER_PLACE_RELEASE_DESCEND cuRobo` 이후 정상 `Plan OK` 확인

### 필수
- [ ] `VERIFY_GRASP` 서비스 연결 — 현재 `read_state service unavailable`로 모든 시도 `GRASP_UNVERIFIED`
- [ ] KPI 수집 시작 (`label_harvest_attempt.py`) — pick+place 안정화 후
  - `measured_tcp_plan_only_hold`를 `terminal_events`에 추가 필요

### 중기
- [ ] 비전 타겟 일관성 확인 (x=-345 vs -401mm 편차 원인)
- [ ] `GRIPPER_LEN` 실측 재확인

### 장기 (NW/NE 대응)
- [ ] point cloud 기반 잎/줄기 동적 장애물 등록 → cuRobo 경로 계획 정확도 향상
- [ ] NW/NE 셀 스캔 및 수확 모션 파라미터 재조정

---

## 7. 알려진 한계

- `GRASP_UNVERIFIED`: read_state 서비스 미연결. 성공 여부는 사람 관찰로만 판단.
- Y-clamp (672mm): FK calibration drift 미보정. raw_Y 적응 진입으로 보상 중.
- 잎/줄기 geometry 없음: 밀도 높은 구역에서 접근 경로 간섭 예측 불가.
- SW 셀 기준 파라미터: NW/NE는 별도 튜닝 필요.
