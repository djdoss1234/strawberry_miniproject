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
