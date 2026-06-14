# Harvest Motion Session - 2026-06-12

## Slot0 place reference candidate

자동 marker-place orientation sampling은 IK에 성공하더라도 J4가 크게 회전하고
계란판 위가 아닌 공중 자세를 선택했다. 이를 해결하기 위해 사용자가 계란판
slot0 근처에서 직접 확인한 place 전용 기준 자세를 기록한다.

현재 이 자세는 `above` 또는 `release`로 확정하지 않은 **수동 티칭 기준 후보**다.
검증 전에는 자동 release 목표로 사용하지 않는다.

```yaml
name: slot0_reference_candidate_20260612
frame: base
joints_deg: [5.34, 3.05, 124.87, 179.46, -3.46, 93.51]
posx_mm_deg: [441.65, 41.20, 233.76, 5.30, 131.37, -87.05]
classification: unverified_place_reference
```

### 기존 실패와의 차이

- 이전 자동 preview는 tray-view 자세에서 임의 orientation 후보를 탐색했다.
- IK 성공만으로 후보를 선택해 J4 급회전과 부적절한 공중 자세가 발생했다.
- 새 후보는 실제 계란판 근처에서 사람이 확인한 관절 브랜치이므로, 향후
  marker slot 목표를 계획할 때 preferred/reference joint pose로 사용할 수 있다.

### 다음 검증

1. 빈 그리퍼, 저속, release 비활성 상태에서 이 자세로 이동 가능 여부를 확인한다.
2. 파츠 끝단의 slot0 중심 정렬과 계란판 clearance를 육안 확인한다.
3. 이 자세가 `above`인지 `release`인지 분류한다.
4. 안전한 `above`와 `release`를 각각 확보한 뒤에만 자동 place에 연결한다.
5. 기준 자세 대비 joint delta와 J4 회전량을 제한하여 엉뚱한 IK branch를 거부한다.

### 고정 기준 pose preview 구현

`use_taught_slot0_place_reference:=true`일 때 기존 임의 orientation sampling을
우회하고 다음 고정 경로를 사용하도록 구현했다.

첫 preview 이후 사용자가 overview와 tray-view 경유 중 계란판에 딸기가 끌리는
문제를 확인했다. 고정 slot0 자세에는 marker 촬영 자세가 필요하지 않으므로
경유점을 제거했다.

```text
pick retreat
 -> taught slot0 place reference 직행
 -> preview hold 또는 명시 승인 시 즉시 gripper release
 -> pick-start scan pose 직접 복귀
```

직행 경로는 cuRobo joint-space planning과 기존 swing/operational-limit 검사를
통과한 경우에만 실행한다. 놓은 직후 tray-view로 복귀하지 않아 계란판 근처에서
딸기를 끄는 동작도 제거했다.

첫 직행 실기 시도에서는 SW retreat의 J1 `153.4°`에서 slot0의 J1 `5.3°`까지
필요한 `148.1°` 회전을 수확 접근용 J1 제한 `75°`가 거부했다. 이는 IK 실패가
아니라 작업 구간별 제한을 구분하지 않은 정책 문제였다.

- 수확 접근의 J1 `75°` 제한은 유지한다.
- 고정 slot0 transfer에만 J1 최대 `170°`를 허용한다.
- operational joint limit과 J4/J6 spline-jump 검사는 계속 적용한다.

## Bent-stem grasp target correction

줄기가 꺾인 딸기에서 그리퍼가 줄기 옆으로 접근해 빗겨 파지하는 현상이
관찰되었다.

원인은 fusion target이 파지점 근처의 줄기 방향이 아니라 `KP0 -> KP2` 전체
줄기의 직선(chord)을 따라 KP0에서 10mm 이동하도록 계산된 점이다. 꺾인 줄기에서는
이 chord가 실제 KP0 바로 위 줄기에서 측면으로 벗어난다.

수정:

- 기본 파지 목표 방향을 `KP0 -> KP1` 국소 줄기 방향으로 변경했다.
- 기존 `KP0 -> KP2` 방식은 `stem_grasp_direction_mode` 파라미터로 되돌릴 수 있다.
- `stem_bend_angle_deg`와 실제 사용한 target source를 runtime JSONL에 기록한다.
- planner의 벽 수직 직선 접근은 유지한다. 줄기 방향 quaternion을 직접 접근축으로
  사용하면 파츠가 측면 진입해 잎과 충돌할 수 있어 아직 실행에 반영하지 않는다.

Place는 이 파지 정확도 검증 동안 비활성으로 실행한다.

## Current harvest sequence and intended grasp point

### Current harvest sequence

```text
SW taught scan pose
 -> seg model ripe filtering
 -> pose model KP0/KP1/KP2 detection
 -> stable target tracking and selection
 -> local stem grasp target generation
 -> cuRobo pre-approach planning
 -> TOOL +Z straight final approach
 -> gripper close
 -> BASE -Z 40mm detach pull
 -> TOOL -Z straight reverse retreat
 -> place disabled: return to SW pick-start scan pose
```

### Keypoint meaning

```text
fruit
  |
 KP0: stem base nearest to fruit
  |
 KP1: local stem midpoint immediately above KP0
   \
   KP2: farther stem direction reference
```

The intended physical grasp point is the thin stem approximately `10~20mm`
above KP0. The stem alone should enter the center between both extension parts.
The gripper must not clamp the fruit body or the wide calyx/leaf area.

For bent stems, the target must follow the local `KP0 -> KP1` direction rather
than the full `KP0 -> KP2` chord.

### Z-bias verification required

The fusion node currently generates a KP-based target and then adds:

- `grasp_target_base_z_trim_m = +10mm`

The planner subsequently adds:

- `GRASP_Z_BIAS = +20mm`

Therefore, the final planner target may be shifted higher than the intended
local stem point by a combined base-Z correction. This is a confirmed code-path
observation, but the physical effect has not yet been isolated experimentally.

Next validation:

1. Test the new KP0->KP1 local target with place disabled.
2. Compare the visual grasp point against KP0+10~20mm.
3. If the target is too high, remove or parameterize the planner-side
   `GRASP_Z_BIAS` instead of changing both layers simultaneously.

## Intermittent gripper close failure diagnosis

실제 파지 위치에 정확히 도달했지만 간헐적으로 그리퍼가 닫히지 않는 현상이
관찰되었다. 기존 planner는 `/dsr01/gripper/close` Trigger를 호출한 뒤 응답의
성공 여부, timeout, 오류 메시지를 확인하지 않았다.

또한 gripper service 내부의 flange serial open/write 재시도는 10초 이상 걸릴 수
있지만 planner는 10초 후 대기를 종료했다. 따라서 close가 실패하거나 아직 처리
중이어도 detach/retreat 단계로 진행할 수 있었다.

수정:

- Trigger service 응답의 `success`와 메시지를 JSONL에 기록한다.
- close 응답 대기를 20초로 늘리고 실패 시 1회 재시도한다.
- 두 번 모두 실패하면 BASE -Z detach를 실행하지 않는다.
- close 실패 시 접근 경로를 직선 역진하고 `GRIPPER_CLOSE_FAILED`로 기록한다.
- read-state 대기 시간도 20초로 늘렸다.

향후 로그에서 `GRIPPER_CLOSE success`와 `trigger_result`를 확인하면, 실제 close
명령 성공 여부와 이후 read-state 실패를 구분할 수 있다.

## Slot0 single-shot place validation

개별 딸기 pick이 대체로 가능해진 뒤 고정 slot0 실제 place 검증을 재개한다.
현재 검증된 고정 place 자세는 slot0 하나뿐이다. slot0 배치 성공 후
`/pick_complete`를 발행하면 다음 딸기를 수확한 뒤 검증되지 않은 marker slot1
경로로 넘어갈 수 있으므로, 기본적으로 첫 place 직후 자동 시퀀스를 잠근다.

```text
pick -> detach -> retreat -> taught slot0 direct transfer -> release -> HOLD
```

`hold_after_taught_slot0_place:=true`가 기본값이며, slot1 이후 경로가 검증되기
전에는 끄지 않는다.

## Relocated tray pose and limited-open release

계란판을 로봇에서 더 멀리 이동하고, 파지 상태의 딸기와 계란판 홈 방향이
수평에 가깝도록 tray-view와 slot0 release를 다시 티칭했다.

### New tray-view

```yaml
joints_deg: [-1.02, 0.11, 97.09, 175.94, -31.34, 93.42]
posx_mm_deg: [505.56, -15.35, 423.49, 176.29, -128.45, 88.27]
```

### New slot0 release

```yaml
joints_deg: [4.43, 51.79, 119.38, 175.95, 80.84, 93.42]
posx_mm_deg: [519.95, 52.39, 65.58, 8.43, 90.35, -87.20]
```

기존 slot0 reference는 새 계란판 위치에서 무효이며 새 값으로 교체했다. 실제
고정 place 실행은 검증된 joint pose를 사용한다. controller TCP와 measured
grasp TCP의 convention 차이가 있으므로 기록된 `posx z=65.58mm`만으로 collision
clearance를 판단하지 않는다.

Place release 시 `/gripper/open`으로 완전히 열지 않고, 접근 시 개도와 동일한
`position_cmd=600`을 사용한다. 이를 통해 딸기를 홈에 내려놓을 때 파츠가 과도하게
벌어지거나 계란판과 간섭하는 것을 줄인다.

### Slot0 vertical place approach

새 slot0 release 자세는 계란판과 그리퍼를 수평으로 맞추지만, 낮은 release
자세로 직접 이동하면 파지한 딸기 밑부분이 계란판에 먼저 닿는다. 따라서 별도
Above 자세를 수동 티칭하지 않고, 검증된 release joint 자세의 cuRobo FK 위치에서
`BASE +Z 120mm`인 Above 목표를 자동 생성한다.

```text
pick retreat
 -> cuRobo plan to auto-generated Slot0 Above
 -> BASE -Z 120mm vertical descend
 -> position_cmd=600 release
 -> BASE +Z 120mm vertical ascend
 -> HOLD
```

계란판 근처 수직 하강/상승 속도는 초기 `20mm/s` 확인 후 `40mm/s`로 조정했다.
table/tray collision
objects가 아직 비활성화되어 있으므로 첫 실기 검증은 release disabled 상태에서
Above 위치와 파지한 딸기의 하단 clearance를 먼저 확인한다.

첫 Above 실기 시도는 SW retreat의 J1 `135.0°`에서 Above J1 `4.4°`로 이동하는
정상적인 place transfer를 일반 수확용 J1 swing 한계 `75°`가 거부했다. 수확
접근의 제한은 유지하고, 고정 Slot0 Above Cartesian plan에만 기존 place transfer
한계인 J1 `170°`를 적용했다. operational joint limit와 spline-jump 검사는 계속
유지한다.

### Gripper close ACK false-positive

실기 로그에서 `/gripper/close`가 `Gripper position set: 700` 성공을 반환했지만
실제 그리퍼가 닫히지 않았고, 이어진 `read_state`도 실패했다. close 서비스 성공은
실제 파지가 아니라 flange serial write 요청 성공만 의미한다. 따라서 close ACK
이후 파지 검증 결과가 `GRASP_UNVERIFIED`이면 close 명령을 한 번 재전송하고 상태를
다시 읽도록 보강했다. 상태 읽기가 계속 실패하면 실제 파지 성공을 선언할 수 없다.

첫 실기 검증 명령은 반드시 `execute_marker_place_release:=false`로 실행한다.

## Bringup YAML parsing incident

`bringup.launch.py` 순정 파일은 변경하지 않았다. 2026-06-11 실측 TCP link를
추가하면서 URDF XML 주석에 포함된 `center: 10mm` 문자열을 ROS Humble launch가
YAML mapping으로 오해해 bringup이 실패했다. 주석의 콜론만 제거하여 실측 TCP
구조를 유지한 채 launch parsing 문제를 해결했다.

## End-of-day status: first observed Slot0 pick-and-place

### Experimentally observed result

2026-06-12 실기에서 SW 딸기 한 개를 수확한 뒤, 재티칭한 고정 Slot0에 배치하는
전체 시퀀스가 처음으로 완료되었다. 사용자가 실제 Slot0 안착을 육안으로 확인했다.

근거 runtime log:

```text
logs/runtime/2026-06-12/
curobo_planner_node_20260612T184149-ec80505c.jsonl
```

관찰된 시퀀스:

```text
SW target 수신
 -> cuRobo pre-approach
 -> TOOL +Z 직선 접근
 -> gripper close
 -> BASE -Z detach
 -> TOOL -Z retreat
 -> cuRobo Slot0 Above transfer
 -> BASE -Z 120mm descend
 -> position_cmd=600 release
 -> BASE +Z 120mm ascend
 -> single-shot HOLD
```

로그 기준 `pick_sequence_start`부터 HOLD까지 약 `53.6초`가 걸렸다. 이는 최적화
전 단일 run이며 반복 성능 수치가 아니다. Slot0 하강과 상승은 모두 성공으로
기록되었고, `marker_place_complete`와 `TAUGHT_SLOT0_PLACE_COMPLETE_HOLD`가
발행되었다.

### Important evidence distinction

- **육안 관찰:** 실제 딸기 1개가 Slot0에 정상 안착했다.
- **자동 로그:** `PLACE_SEQUENCE_COMPLETE_UNVERIFIED`로 기록되었다.
- **파지 검증:** gripper close와 close 재시도 명령은 성공 응답을 받았지만,
  `read_state service error` 때문에 `GRASP_UNVERIFIED` 상태였다.

따라서 이번 결과는 “Slot0 pick-and-place 실기 성공 사례 1건”으로 기록하되,
자동 검증 성공률 또는 반복 Place 성공률로 주장하지 않는다.

### Problems found and resolved today

| 문제 | 원인 | 적용한 해결 |
| --- | --- | --- |
| 이동된 계란판과 기존 pose 불일치 | 예전 tray-view/Slot0 pose 사용 | 새 tray-view와 Slot0 release 재티칭 |
| 낮은 Slot0로 직접 이동 중 딸기 하단 접촉 | release 자세를 transfer 목표로 직접 사용 | release FK 기준 `BASE +Z 120mm` Above 자동 생성 |
| Slot0 Above 계획 거부 | 일반 수확용 J1 swing `75°` 제한이 정상 place transfer 차단 | Slot0 transfer에만 J1 `170°` 허용 |
| Place 시 파츠 과도 개방 | `/gripper/open` 완전 개방 | `position_cmd=600` 제한 개방 |
| 그리퍼 close 거짓 성공 가능성 | close 성공은 serial write ACK이며 실제 상태 미확인 | `GRASP_UNVERIFIED` 시 close 1회 재전송 및 재검증 |
| 다음 딸기 자동 진행 위험 | 나머지 slot 경로 미검증 | Slot0 완료 직후 single-shot HOLD 유지 |

### Remaining problems

- 실제 그리퍼 position/current 읽기가 계속 실패하여 파지 성공 자동 판정이 불가능하다.
- table 및 tray collision object가 비활성 상태라 계란판 주변 안전 여유를 planner가
  완전히 검증하지 못한다.
- 현재 성공한 고정 Slot0와 marker 기반 최신 Slot0 계산 좌표가 크게 달라 marker
  좌표를 나머지 슬롯에 바로 사용할 수 없다.
- 전체 작업 시간 약 `53.6초`는 길며, 그리퍼 통신 대기·재시도와 transfer 시간이
  주요 단축 대상이다.
- Slot0 성공은 1회 관찰 사례이며 반복 성공률은 아직 측정하지 않았다.

### Tray configuration outside this repository

`~/Downloads/share_tray/robot_poses.yaml`의 `egg_tray_view`도 아래 새 좌표로
갱신했고, `run_tray_localization.py` 로더가 새 관절값을 읽는 것을 확인했다.

```yaml
joints_deg: [-1.02, 0.11, 97.09, 175.94, -31.34, 93.42]
task_pose: [505.56, -15.35, 423.49, 176.29, -128.45, 88.27]
```

`share_tray` 디렉터리는 별도 Git 저장소가 아니므로 이 변경은 본 저장소 커밋에
포함되지 않는다.

### Next session priorities

1. gripper `read_state` 통신을 안정화하고 실제 파지 상태 자동 판정을 검증한다.
2. Slot1과 Slot3만 추가 티칭하여 열·행 방향 벡터를 계산한다.
3. `Slot(r,c) = Slot0 + r * row_vector + c * col_vector`로 15개 release 목표를
   자동 생성한다.
4. 빈 그리퍼로 Slot2, Slot12, Slot14 모서리 위치와 Above/수직 하강을 검증한다.
5. 검증된 빈 슬롯 순서로 한 개씩 Place하고, 각 시도마다 사람 라벨과 JSONL을
   함께 기록한다.
6. 이후 SW 연속 수확과 최소 30회 반복 KPI 측정을 시작한다.

## 2026-06-14 — Slot1/Slot3 티칭 기반 15-slot 격자 생성

계란판 인덱스는 다음과 같이 확정했다.

```text
Slot0   Slot3   Slot6   Slot9   Slot12
Slot1   Slot4   Slot7   Slot10  Slot13
Slot2   Slot5   Slot8   Slot11  Slot14
```

추가 실측 티칭값:

| 기준 | Joint deg | TCP BASE `[x,y,z,rx,ry,rz]` |
| --- | --- | --- |
| Slot1 | `[6.02, 50.74, 128.89, 177.59, 89.27, 92.82]` | `[460.24, 55.83, 66.47, 8.43, 90.36, 87.20]` |
| Slot3 | `[-3.93, 52.00, 120.58, 167.53, 82.38, 94.40]` | `[511.91, 1.83, 63.12, 8.43, 90.37, -87.20]` |

Slot0 대비 위치 벡터:

```text
Slot0 -> Slot1: [-59.71, +3.44, +0.89] mm, 길이 59.82 mm
Slot0 -> Slot3: [ -8.04,-50.56, -2.46] mm, 길이 51.25 mm
두 벡터 cosine 약 0.099: 거의 직교
```

Slot1의 티칭 손목 방향은 Slot0/Slot3과 반대이므로 위치 벡터만 사용한다. 생성된
15개 슬롯은 검증된 Slot0 cuRobo FK orientation을 공통으로 유지한다. 또한
controller TCP와 cuRobo measured TCP 기준점 차이를 피하기 위해, Slot0의 cuRobo
FK 위치에 실측 BASE 위치 차이만 더해 각 목표를 생성한다.

예상 모서리 release 위치:

| Slot | TCP BASE 예상 위치 `[x,y,z] mm` |
| --- | --- |
| Slot2 | `[400.53, 59.27, 67.36]` |
| Slot12 | `[487.79, -149.85, 55.74]` |
| Slot14 | `[368.37, -142.97, 57.52]` |

코드에는 `initial_place_slot_index` 파라미터와 15-slot 순차 증가 로직을 추가했다.
모든 슬롯을 사용하면 `TAUGHT_TRAY_FULL`로 자동 진행을 정지한다.

실기 적용 전 검증 순서:

1. `execute_marker_place_release:=false`로 Slot2, Slot12, Slot14의 Above 위치를 확인한다.
2. 계란판 및 테이블과의 여유를 육안 확인한다.
3. 각 모서리에서 수직 하강을 저속 단일 실행으로 검증한다.
4. 검증 후에만 `hold_after_taught_slot0_place:=false`로 연속 Place를 허용한다.

현재 table/tray collision object가 비활성 상태이므로 모서리 검증 전 연속 실기
Place는 수행하지 않는다.

### Corner Above preview commands

현재 preview는 독립적인 tray 이동 명령이 아니라, 한 번의 pick/retreat 이후 지정
슬롯 Above로 이동하는 전체 시퀀스다. 따라서 딸기를 잡은 상태에서 하단 clearance도
함께 확인해야 한다. 각 preview 후 planner가 HOLD되므로 다음 슬롯은 planner를
재시작해서 검증한다.

첫 검증은 Slot2부터 시작한다.

```bash
source ~/doosan_ws/install/setup.bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p measured_tcp_plan_only:=false \
  -p enable_marker_place_sequence:=true \
  -p use_taught_slot0_place_reference:=true \
  -p initial_place_slot_index:=2 \
  -p execute_marker_place_release:=false \
  -p allow_unverified_grasp_place:=true
```

Slot2 Above를 확인한 뒤 같은 명령의 `initial_place_slot_index`만 각각 `12`, `14`로
바꿔 반복한다. 예상 로그는 다음과 같다.

```text
TAUGHT_TRAY_GRID_PLACE active: slot=2
TAUGHT_TRAY_SLOT2_ABOVE generated from Slot0 FK + grid offset
TAUGHT_TRAY_SLOT2_PLACE_PREVIEW_HOLD
```

`execute_marker_place_release:=false`이므로 수직 하강과 그리퍼 release는 실행하지
않는다. Above 위치에서 E-stop 여유, 딸기 하단과 계란판 간격, 로봇 링크와 테이블
간격을 확인한 뒤 다음 검증으로 넘어간다.

### 2026-06-14 Slot2 preview 첫 시도 실패 원인

첫 Slot2 preview는 Place에 도달하기 전 Pick 직선 진입에서 중단되었다.

```text
MoveSplineJoint 요청: 120 deg/s, 180 deg/s2, requested time 0.75 s
실제 MoveSplineJoint 응답 시간: 약 34.7 s
MoveLine 요청: TOOL +Z 180 mm, 50 mm/s
고정 service timeout: 30 s
결과: FINAL_APPROACH_STRAIGHT timeout -> gripper close 미실행
```

`/dsr01/realtime/read_data_rt`로 확인한 실제 컨트롤러
`operation_speed_rate`는 `10%`였다. 따라서 180 mm 직선 진입의 실제 속도는
명목상 50 mm/s가 아니라 약 5 mm/s이며, 예상 시간 약 36초가 고정 30초 timeout을
초과했다. 그리퍼 고장이 아니라 직선 진입 실패 시퀀스가 안전하게 close를 생략한
것이다.

수정:

- MoveLine timeout을 고정 30초에서 `거리 / 명령 속도` 기반으로 계산한다.
- 최소 운전 속도율 10%까지 고려하고 10초 여유를 추가한다.
- 빠른 실기 검증 시에는 티치펜던트 운전 속도율을 먼저 확인한다.

속도율은 ROS 서비스 `/dsr01/motion/change_operation_speed`로 관리한다. 이후
실기에서는 `100%`로 설정하고, 사용자가 E-stop을 준비한 상태에서 모서리
clearance를 확인한다.

## 2026-06-14 — Slot2 preview 결과와 Pick 정책 단순화

ROS 서비스 `/dsr01/motion/change_operation_speed`로 컨트롤러
`operation_speed_rate`를 `100%`로 변경한 뒤 Slot2 preview를 재실행했다.

### Slot2 preview 결과

Slot2 Place 경로는 실패하지 않았다.

```text
TAUGHT_TRAY_SLOT2_ABOVE goal: [657.7, 97.3, 185.8] mm
cuRobo plan latency: 1.63 s
MoveSplineJoint result: success
TAUGHT_TRAY_SLOT2_PLACE_PREVIEW_HOLD
```

`execute_marker_place_release:=false`였기 때문에 Above 도달 후 의도적으로 정지했다.
이는 Place 경로 실패가 아니라 release/하강 전 안전 preview 성공이다.

### 확인된 시간 병목

| 구간 | 관찰 시간/원인 |
| --- | --- |
| pre-approach 후보 탐색 | 약 4.3초, `-10°/-5°` 후보 spline-jump 거부 후 수평 후보 성공 |
| pre-approach 실행 | 약 3.2초 |
| final MoveLine | 약 4.6초 |
| close 및 자동 파지 검증 | 약 13.3초, `read_state` 실패 후 close 재전송 포함 |
| Slot2 Above 계획 | 약 1.6초 |

### 파지 목표 및 속도 정책 변경

사용자 실기 관찰에 따라 “줄기보다 위에서 잡은 뒤 내리기” 대신, 줄기의 국소
중간점인 KP1 근처에서 바로 닫고 BASE -Z로 분리하도록 변경했다.

```text
이전:
fusion KP0->KP1 최대 10mm + fusion BASE Z 10mm + planner BASE Z 20mm

변경:
fusion KP0->KP1 구간 80% 지점(KP1 근처)
+ fusion BASE Z 0mm
+ planner BASE Z 0mm
-> close
-> BASE -Z detach pull
```

꺾인 줄기에서도 `KP0 -> KP1` 국소 방향을 사용하므로 `KP0 -> KP2` 전체 chord로
인한 측방 편차를 줄인다.

계획 및 대기 시간 단축:

- 검증된 수평 `0°` orientation을 첫 후보로 이동했다.
- pre-approach IK seed를 `48 -> 24`로 줄이고 실패 시 나머지 orientation으로
  fallback한다.
- 실제 상태 읽기 실패만으로 close를 재전송하지 않도록 변경했다.
- close 후 안정화 대기를 `1.5초 -> 0.3초`로 줄였다.
- 파지 검증 read timeout을 `20초 -> 5초`로 줄였다.

주의: 위 변경은 코드/빌드 검증 상태이며 KP1 근처 실제 파지 정확도와 planning
latency 개선량은 다음 단일 SW 실기 run에서 확인해야 한다.

### KP1 근처 파지 정책 실기 적용 확인

근거 runtime logs:

```text
logs/runtime/2026-06-14/strawberry_fusion_node_20260614T151243-0d992d6a.jsonl
logs/runtime/2026-06-14/curobo_planner_node_20260614T151240-6aa797eb.jsonl
```

이번 target의 keypoint와 실제 생성 목표:

```text
KP0 = [-124.45, 742.34, 508.94] mm  # 과실에 가까운 줄기 시작점
KP1 = [-125.43, 742.12, 527.73] mm  # 줄기 국소 중간점
KP0->KP1 길이 = 18.82 mm
선택 offset = 15.05 mm = KP0->KP1 구간의 80%
fusion 목표 = [-125.24, 742.17, 523.97] mm
planner GRASP_Z_BIAS = 0 mm
```

즉 파츠 중심이 목표로 하는 물리 지점은 KP0 바로 위나 KP2가 아니라, **KP0에서
KP1 방향으로 80% 올라간 가는 줄기 부분**이다. 꺾인 줄기에서도 전체 줄기
`KP0->KP2` chord가 아니라 파지점 주변의 `KP0->KP1` 국소 방향만 사용한다.

현재 Pick 시퀀스:

```text
1. fusion이 ripe 과실과 KP0/KP1/KP2를 매칭
2. KP0->KP1 구간 80% 지점을 안정화해 pick target 발행
3. planner가 같은 높이에서 수평 6cm pre-approach 계획
4. MoveSplineJoint로 pre-approach 이동
5. TOOL +Z 180mm 직선 접근
6. 해당 줄기 위치에서 gripper close
7. BASE -Z 40mm로 줄기 분리
8. TOOL -Z 180mm 후퇴
9. cuRobo로 지정 tray slot Above 이동
10. preview이면 Above에서 HOLD, release 승인 시 수직 하강/해제/상승
```

이번 run에서는 첫 수평 orientation 후보가 바로 성공하여 pre-approach 후보
reject가 `0건`이었고, 이전 run의 약 4.3초 후보 탐색이 약 2.0초 단일 계획으로
줄었다. close 재전송도 제거되어 close 시작부터 verify 결과까지 약 5.5초로
감소했다.

Slot2 Place는 실패하지 않았다.

```text
Slot2 Above cuRobo planning: 1.90 s, success
MoveSplineJoint: success
execute_marker_place_release=false
result: TAUGHT_TRAY_SLOT2_PLACE_PREVIEW_HOLD
```

따라서 실제 Slot2 수직 하강과 release를 실행하려면 동일 명령에서
`execute_marker_place_release:=true`로 명시해야 한다. 모서리 하강 clearance를
아직 확인하지 않았다면 preview 상태를 유지한다.
