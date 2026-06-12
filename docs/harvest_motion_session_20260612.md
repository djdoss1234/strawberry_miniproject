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

계란판 근처 수직 하강/상승 속도는 `20mm/s`로 제한한다. table/tray collision
objects가 아직 비활성화되어 있으므로 첫 실기 검증은 release disabled 상태에서
Above 위치와 파지한 딸기의 하단 clearance를 먼저 확인한다.

첫 실기 검증 명령은 반드시 `execute_marker_place_release:=false`로 실행한다.

## Bringup YAML parsing incident

`bringup.launch.py` 순정 파일은 변경하지 않았다. 2026-06-11 실측 TCP link를
추가하면서 URDF XML 주석에 포함된 `center: 10mm` 문자열을 ROS Humble launch가
YAML mapping으로 오해해 bringup이 실패했다. 주석의 콜론만 제거하여 실측 TCP
구조를 유지한 채 launch parsing 문제를 해결했다.
