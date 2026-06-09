# Harvest Motion Session - 2026-06-09

## Leftmost Strawberry Top-Down Approach

### 배경

맨 왼쪽 딸기(`base_link x < -300mm`)는 기존 수평 접근에서 cuRobo가 주로
`J4 ~= 270deg` ELBOW_UP branch를 선택했다. Pitch-down 후보는 J4 operational
limit 또는 spline jump 검사에서 거부되고, 수평 후보는 줄기 주변의 수평 잎을
밀면서 진입하는 문제가 있었다.

### 구현 상태

`scripts/curobo_planner_node.py`에 맨 왼쪽 target 전용 실험적 top-down 모드를
추가했다. 기존 수평 접근은 `x >= -300mm`에서 변경하지 않았다.

```text
LEFTMOST_TOP_DOWN_X_THRESHOLD_M = -0.300
TOP_DOWN_QUAT_WXYZ = [0.0, 0.70710678, 0.70710678, 0.0]
TOP_DOWN_PRE_APPROACH_Z_OFFSET_M = 0.150
TOP_DOWN_WALL_CLEARANCE_M = 0.050
```

Quaternion 축 검증:

```text
TOOL +Z -> world [0, 0, -1]  # 위에서 아래로 하강
TOOL +X -> world [0, 1, 0]   # jaw 축이 벽 방향
```

Top-down 목표는 물리 fingertip이 아니라 `ee_link` 기준으로 계산한다.

```text
stem_clear = stem + [0, -0.05, 0]
ee_pre     = stem_clear + [0, 0, GRIPPER_LEN + 0.15]
ee_grasp   = stem_clear + [0, 0, GRIPPER_LEN]
```

실행 순서:

```text
leftmost target
 -> cuRobo: current joints -> top-down pre-approach
 -> cuRobo: grasp endpoint IK/collision/branch 사전 검증
 -> MoveSplineJoint: pre-approach 실행
 -> MoveLine TOOL +Z: world -Z 방향 150mm 수직 하강
 -> gripper close / VERIFY_GRASP
 -> MoveLine TOOL -Z: world +Z 방향 150mm 수직 상승
 -> optional guarded marker place
 -> pick-start scan pose 복귀
```

### Fallback 및 안전 경계

- top-down pre-approach 또는 grasp endpoint가 **계획 단계에서 실패**하면 기존
  leftmost 수평 접근으로 fallback한다.
- top-down 실제 모션을 한 번이라도 시작한 뒤 실패하면 수평 경로로 다시 진입하지
  않는다.
- top-down quaternion은 교시값이 아닌 이론값이다.
- 잎/줄기는 cuRobo collision world에 없으므로 top-down도 무충돌을 보장하지 않는다.
- 첫 실기 검증은 clear-space, low-speed, single-target, place 비활성 조건에서
  수행해야 한다.

### 실기 전 검증 항목

1. 시작 로그에서 `tool_z=[0,0,-1]`, `tool_x=[0,1,0]` 확인
2. RViz/offline에서 pre-approach가 target 위쪽이며 벽 앞 50mm인지 확인
3. 그리퍼를 닫지 않은 상태로 pre-approach까지만 저속 검증
4. TOOL +Z가 실제 world -Z로 움직이는지 짧은 거리에서 확인
5. clear-space 150mm 하강/상승 후에만 단일 딸기 파지를 허용

현재 단계는 **코드 및 이론 축 검증 완료, 실기 미검증**이다.

## Leftmost Grasp Verification Verdict

### 판정

맨 왼쪽 딸기를 잘 잡았다고 관찰한 실행은 **top-down 파지 성공이 아니다**.
runtime JSONL의 실행 경로를 대조한 결과, top-down은 계획 단계에서 실패했고
기존 수평 접근 fallback이 실제로 실행되었다.

검증 대상:

```text
run_id: 20260609T103947-913da046
git_commit: 2665e17
raw target: x=-354.9mm, y=715.9mm, z=537.7mm
```

실행 증거:

```text
top_down_attempt
 -> approach_dir=[0, 0, -1]
 -> top_down_fallback reason=pre_approach_plan_failed
 -> top_down_fallback_horizontal x_correction=+10mm
 -> grasp_variant=base X-axis -5deg
 -> approach_dir=[0, 0.9962, -0.0872]
 -> TOOL +Z 130mm 수평 중심 진입
```

따라서 실제 접근은 벽 방향 `+Y`가 주성분이고 아래쪽 성분이 약 `-8.7%`인
수평 `-5deg` 접근이었다. 위에서 아래로 향하는 `[0, 0, -1]` top-down 경로는
로봇에 실행되지 않았다.

### 해결된 문제

**맨 왼쪽 과실에 접근하지 못하던 문제는 계획 실패를 감지하고 수평 `-5deg`
fallback으로 전환하는 정책을 통해 실기 관찰상 파지 가능한 상태가 되었다.**

이 결과는 다음 범위에서 해결된 것으로 기록한다.

- 맨 왼쪽 target을 별도 분기로 식별
- 실행 불가능한 top-down pre-approach를 실제 모션 전에 차단
- fallback 시 target X를 `+10mm` 보정
- 수평 `-5deg` 경로로 실제 gripper close 및 사용자 관찰상 파지 도달

### 아직 해결되지 않은 항목

- top-down 접근 자체는 `pre_approach_plan_failed`로 아직 실행 성공 사례가 없다.
- `verify_grasp=GRASP_UNVERIFIED`, `present_position=-1`이므로 센서 기반 파지 성공
  증거는 없다.
- 해당 실행의 detach도 `DETACH_UNVERIFIED`다.
- 따라서 정량 KPI에는 `SUCCESS`로 집계하지 않고
  `experimentally observed grasp / sensor unverified`로 분류한다.

## Guarded Downward Detach Retreat

### 실기 관찰과 적용 범위

사용자는 leftmost 대상에서 파지가 잘 된 것을 관찰했다. 그러나 해당 실행의
runtime JSONL을 확인하면 실제 파지 경로는 top-down이 아니라 기존 수평 접근의
`-5deg` pitch fallback이었다.

따라서 이번 변경은 **실제로 사용된 수평 접근 경로의 파지 후 retreat**에만
적용한다. Top-down 접근이 실제로 실행된 경우에는 파지 후 더 아래로 내려가지
않고 기존처럼 TOOL `-Z`, 즉 world `+Z` 방향으로 상승한다.

### 변경된 수평 접근 후 분리 정책

기존에는 파지 진입 경로를 그대로 정면 후진했다. 이제는 다음 조건을 모두
만족할 때만 `base_link -Z` 방향으로 `100mm` 내려서 줄기 분리를 시도한다.

```text
horizontal grasp complete
 -> detected-neighbor swept corridor 검사
 -> minimum Z 검사
 -> cuRobo 하강 endpoint/path candidate 검증
 -> MoveLine BASE REL [0, 0, -100mm] 실행
 -> place 또는 다음 복귀 단계
```

하강 corridor는 파지한 과실 반경 `45mm`, 이웃 과실 sphere 반경 `30mm`,
추가 여유 `15mm`를 합친 `90mm`를 요구한다. 파지 시작 시점에 등록한 이웃 과실
snapshot을 사용하므로 로봇 이동 중 perception 출력이 바뀌어도 검사 기준이
갑자기 변하지 않는다.

### Fallback 및 안전 경계

- 이웃 과실 corridor가 막혔거나 cuRobo endpoint 검증이 실패하면 기존
  정면 후진 retreat으로 fallback한다.
- 하강 MoveLine이 시작된 뒤 실패하면 추가 fallback 이동을 금지하고 현재
  자세에서 hold한다.
- cuRobo 검증은 하강 endpoint에 도달 가능한 collision-aware 경로 후보를
  확인한다. 실제 실행은 정확한 `base_link -Z` MoveLine이며, 이 직선 경로는
  검출된 과실 swept corridor로 별도 보호한다.
- 잎과 줄기는 현재 cuRobo world 및 corridor 모델에 없으므로 잎/줄기 접촉
  회피는 보장하지 않는다.
- 현재 단계는 **코드 검증 완료, 하강 detach 실기 미검증**이다.

### 실기 확인 로그

하강이 허용되면 다음 로그가 순서대로 보여야 한다.

```text
downward_retreat_corridor_check: clear=true
downward_retreat_endpoint_check: accepted=true
4 downward detach retreat — BASE -Z 100.0mm
DETACH_DOWNWARD_BASE_Z BASE REL xyz=[0.0, 0.0, -100.0]mm
```

하강이 거부되면 `DOWNWARD_RETREAT_BLOCKED` 또는
`DOWNWARD_RETREAT_ENDPOINT_REJECTED` 이후 `RETREAT_STRAIGHT_REVERSE`가 실행된다.

## Leftmost Horizontal Fallback Fine Tuning

### 11:11 실기 관찰

맨 왼쪽 과실에 수평 `-5deg` fallback으로 접근했으나, 전날 파지 관찰 경로보다
아주 약간 오른쪽으로 빗겨가고 진입 깊이가 부족했다.

비교 로그:

| 항목 | 이전 파지 관찰 run | 이번 run |
| --- | ---: | ---: |
| run ID | `20260609T103947-913da046` | `20260609T111119-3d421ffc` |
| raw target X | `-354.9mm` | `-348.5mm` |
| 기존 X correction | `+10mm` | `+10mm` |
| corrected target X | `-344.9mm` | `-338.5mm` |
| 선택 grasp offset | `50mm` | `50mm` |
| 실제 variant | horizontal `-5deg` | horizontal `-5deg` |

이번 detection target이 이전보다 약 `6.4mm` 오른쪽에서 생성된 상태에서 고정
`+10mm` 보정이 다시 적용되어 corrected target이 더 오른쪽으로 이동했다. 또한
`40mm` 깊이 후보가 IK 실패하면 기존에는 바로 `50mm` 후보를 선택하여, 중간의
더 깊은 유효 경로를 검사하지 않았다.

### 조정

```text
LEFTMOST_GRASP_X_CORR_M: +10mm -> +5mm
LEFTMOST_GRASP_RETRY_OFFSETS: [40, 50, 70]mm -> [40, 45, 50, 70]mm
```

- 오른쪽 편차를 줄이기 위해 X 보정을 절반으로 낮췄다.
- `40mm` endpoint가 IK 실패할 때 `45mm` endpoint를 추가로 검증하여, 기존
  `50mm`보다 최대 `5mm` 더 깊게 진입할 수 있게 했다.
- 검증되지 않은 endpoint를 향해 MoveLine을 강제로 연장하지 않는다.
- 다른 과실의 좌표 보정 및 깊이 후보는 변경하지 않았다.

현재 단계는 **코드/빌드 검증 대상이며, 새 `+5mm / 45mm` 설정은 실기 미검증**이다.

## Leftmost Deeper Endpoint Search

### 11:22 실기 결과

`20260609T112156-40cf85e9`에서 새 `45mm` 후보가 실제로 검사되었지만,
`40mm`와 `45mm` endpoint가 모두 `MotionGenStatus.IK_FAIL`이었다. 따라서
planner는 다시 `50mm` stand-off를 선택했고 실제 직선 진입 거리도 기존과 같은
`130mm`였다.

즉, 이전 조정은 후보를 추가했지만 실제 진입 깊이를 바꾸지 못했다.

```text
40mm endpoint -> IK_FAIL
45mm endpoint -> IK_FAIL
50mm endpoint -> Plan OK
GRASP_POSE_REACHED offset=+0.050m
```

### 조정

맨 왼쪽 수평 fallback에만 더 깊은 endpoint 탐색 범위를 확장했다.

```text
LEFTMOST_GRASP_RETRY_OFFSETS = [30, 35, 40, 45, 50, 70]mm
deep endpoint IK seeds = 128
deep endpoint max attempts = 4
deep endpoint timeout = 3.0s
```

- `30mm`가 선택되면 기존 `50mm`보다 `20mm` 더 깊게 진입한다.
- `35mm`는 `15mm`, `40mm`는 `10mm`, `45mm`는 `5mm` 더 깊다.
- 이 강화 탐색은 맨 왼쪽 target의 `30~45mm` endpoint에만 적용한다.
- 모든 깊은 endpoint가 거부되면 `LEFTMOST_DEPTH_LIMITED`를 기록하고 검증된
  `50mm` 이상 경로를 사용한다.
- IK 검증에 실패한 위치로 MoveLine을 강제 연장하지 않는다.

현재 단계는 **코드 검증 대상, 강화된 깊이 탐색 실기 미검증**이다.

## Guarded Extra Advance For Leftmost Horizontal Fallback

### 11:33~11:34 강화 탐색 결과

`d62c894` 기준 실기 로그 2건에서 맨 왼쪽 target의 `30/35/40/45mm`
endpoint는 강화된 IK seed/attempt 설정에서도 모두 실패했고, `50mm` stand-off만
성공했다. 따라서 단순히 깊은 후보를 더 앞에 배치하는 방식으로는 진입 깊이가
늘어나지 않았다.

사용자는 방향은 적절하지만 실제 파지 홈까지 약 `80~100mm` 진입이 부족하다고
관찰했다. 그러나 현재 모델은 `GRIPPER_LEN=160mm`, target/wall
`Y=672mm`, 선택 stand-off `50mm`를 사용한다. 이 모델에서 `80~100mm`를
강제로 더 진입시키면 whiteboard를 약 `30~50mm` 관통하는 계산이 된다.

이는 단순 motion tuning 문제보다는 다음 보정값의 불일치 가능성을 의미한다.

- cuRobo `ee_link`에서 실제 파지 홈까지의 유효 길이
- `GRIPPER_LEN=160mm` 가정
- eye-in-hand/FK 기반 target Y와 `WALL_SURFACE_Y_M`

### 안전 제한 추가

맨 왼쪽 target의 top-down 실패 후 수평 fallback에만 저속 추가 진입을 적용한다.

```text
requested extra advance = 80mm
selected stand-off       = 50mm
wall safety margin       = 20mm
maximum executed advance = 30mm
extra advance velocity   = 10mm/s
```

- 요청값과 실제 실행값은 `LEFTMOST_EXTRA_ADVANCE_CAPPED` 및 runtime JSONL에
  각각 기록한다.
- 추가 진입 후 하강 retreat 기준점을 같은 거리만큼 갱신한다.
- 하강 retreat가 거부되어 직선 역진 fallback을 사용하면 기존 진입 거리와 추가
  진입 거리를 합산하여 정확히 되돌아간다.
- 이 추가 구간은 cuRobo endpoint 검증이 아니라 wall-distance gate만 통과한
  실험 구간이다. 첫 실기 검증은 clear-space, single-target, E-stop 준비 상태에서
  수행해야 한다.

### 다음 필수 검증

`80~100mm` 전체 보정을 적용하기 전, 실제 장비에서 `ee_link` 기준점부터 실제
줄기가 들어가는 파지 홈까지의 TOOL +Z 길이를 측정하여 `GRIPPER_LEN`과 비교해야
한다. 길이 오차가 확인되면 추가 MoveLine 보정보다 robot/tool geometry를 먼저
수정한다.

## Extra 30mm 실기 확인 및 명시적 80mm 검증 모드

### 11:45 실기 결과

`20260609T114452-cc69112c`에서 안전 제한 추가 진입이 실제 실행되었다.

```text
FINAL_APPROACH_STRAIGHT = 130mm
LEFTMOST_EXTRA_ADVANCE  = 30mm
total approach          = 160mm
```

두 MoveLine 모두 controller success였지만 사용자는 실제 파지 홈까지 여전히
약 `80~100mm` 부족하다고 관찰했다. 따라서 추가 이동 명령 누락은 원인이 아니다.

또한 이전 맨 왼쪽 관찰 run과 현재 run은 동일 target이 아니었다.

| run | raw target X | raw target Z | 기본 진입 |
| --- | ---: | ---: | ---: |
| `20260608T192005-cecd7d3e` | -401mm | 569mm | 130mm |
| `20260609T114452-cc69112c` | -345mm | 534mm | 130mm + extra 30mm |

현재 비전 target은 이전보다 약 `56mm` 오른쪽, `35mm` 아래에서 생성되었다.
즉 최근 motion 코드가 기본 진입을 줄인 것이 아니라, target 선택/추정 위치가
달라졌고 실제 TCP/파지 홈 모델 오차도 남아 있다.

### 명시적 wall-model override

실물 간격을 작업자가 직접 확인한 단일 target 검증에서만 요청한 `80mm` 전체
추가 진입을 실행할 수 있도록 다음 ROS parameter를 추가했다.

```bash
-p leftmost_allow_wall_model_override:=true
```

- 기본값은 `false`이며 기존 30mm 안전 제한을 유지한다.
- `true`이면 `leftmost_extra_advance_request_m` 요청값 전체를 저속 실행한다.
- 현재 기본 요청값은 `80mm`이다.
- 모델상 wall overtravel 양과 override 사용 사실을 ERROR 로그 및 runtime
  JSONL에 기록한다.
- 직선 역진 fallback은 `추가 진입 역진 -> 기본 진입 역진` 두 단계로 나누어,
  총 진입 거리가 180mm MoveLine 단일 명령 한도를 넘어도 복귀할 수 있게 했다.

이 모드는 **tool/TCP calibration 수정이 아니라 실기 원인 분리용 임시 검증
모드**다. E-stop 준비, 낮은 속도, 단일 target, whiteboard까지의 실제 간격 확인
없이 자동 반복 실행하면 안 된다.
