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
