# Tool Geometry Measurement - 2026-06-11

## 실측 결과

현재 로봇에 장착된 원래 그리퍼와 연장 파츠를 직접 측정한 값이다.

```text
link_6 / robot flange
  -> original gripper reference: 약 160mm
  -> extension part tips:         약 270mm
  -> effective grasp center:      약 260mm
```

유효 파지점은 파츠 끝단보다 약 `10mm` 뒤쪽으로 정의한다. 따라서 현재 사용할
물리 파지 TCP 후보는 플랜지 기준 약 `260mm`다. 그리퍼 기준 연장 파츠의 유효
길이는 약 `110~120mm`다.

## 현재 좌표계 정의

화이트보드를 로봇 정면에서 바라보는 현재 설치 기준이다.

### `base_link`

```text
base_link +X: 화이트보드 기준 오른쪽
base_link -X: 화이트보드 기준 왼쪽
base_link +Y: 로봇에서 화이트보드 방향
base_link -Y: 화이트보드에서 로봇 방향
base_link +Z: 위
base_link -Z: 아래
```

화이트보드 전면 모델은 `base_link Y=672mm` 부근이다. 파지 후 사용하는
`BASE -Z 40mm` detach pull은 TCP 자세를 유지하고 수직 아래로 당기는 동작이다.

### 현재 수평 파지 자세의 TOOL 좌표계

현재 `WALL_QUAT_WXYZ=[0.497, -0.497, 0.503, 0.503]` 자세에서만 다음 관계가
성립한다.

```text
TOOL +Z: 화이트보드 방향, 정면 진입
TOOL -Z: 로봇 방향, 정면 후퇴
TOOL +X: 아래
TOOL -X: 위
TOOL +Y: 화이트보드를 봤을 때 왼쪽
TOOL -Y: 화이트보드를 봤을 때 오른쪽
```

로봇 wrist 자세가 바뀌면 TOOL X/Y/Z도 함께 회전한다. `base_link` 방향은
고정이지만 TOOL 방향은 고정이 아니다.

### TCP 기준점

```text
link_6 / flange: 로봇 손목 플랜지 중심
legacy software TCP: flange TOOL +Z 160mm
grasp_tcp_link: flange TOOL +Z 약 260mm
part tips: flange TOOL +Z 약 270mm
```

`grasp_tcp_link`는 양쪽 연장 파츠 끝단 사이 중앙에서 약 `10mm` 뒤쪽인 실제
줄기 파지 홈을 의미한다. 현재 새 cuRobo 프로필의 목표 좌표는 이 점이다.

## 현재 소프트웨어 모델과 차이

현재 cuRobo 설정은 다음 상태다.

- `ee_link`: `gripper_rh_p12_rn_base`
- URDF의 `gripper_attach_joint`: `link_6`과 같은 위치
- planner legacy TCP offset: `160mm`
- 실측 flange-to-grasp-center: 약 `260mm`
- 모델 오차: 약 `100mm` 짧음

현재 SW 수확 성공 baseline은 잘못된 `160mm` 모델에 `extra advance`와 grasp
offset을 더해 실물에 맞춘 상태다. 따라서 offset만 즉시 `260mm`로 바꾸면 기존
보정과 중복되어 실제 로봇이 약 `100mm` 다르게 움직일 수 있다.

## 이 오차로 발생한 문제

- 계산상 목표 도달과 실제 파츠 파지 홈 위치가 일치하지 않았다.
- 진입 깊이 부족을 `65~80mm extra advance`로 반복 보정했다.
- 파츠 끝단과 파지 홈이 collision model에 정확히 표현되지 않았다.
- `grasp OK`가 실제 줄기 도달이 아니라 legacy 모델의 목표 도달을 의미했다.
- target depth, wall 위치, TCP 길이 중 어느 값이 원인인지 분리하기 어려웠다.

## 안전한 전환 순서

1. 현재 SW 성공 baseline과 legacy offset `160mm`를 보존한다.
2. URDF에 플랜지에서 원래 그리퍼까지의 실제 transform을 반영한다.
3. 플랜지 기준 약 `260mm`에 명시적인 `grasp_tcp_link`를 추가한다.
4. 연장 파츠와 파지 홈 collision sphere를 실측 치수로 다시 만든다.
5. cuRobo `ee_link`를 `grasp_tcp_link`로 변경한다.
6. `extra advance`, wall override, leftmost 전용 깊이 보정을 비활성화하고
   저속 dry-run으로 검증한다.
7. 검증 후 legacy offset과 불필요해진 임시 보정 코드를 삭제한다.

## 구현된 전환 프로필

`curobo_planner_node.py`의 기본 프로필은 `measured_tcp_260mm`다.

```bash
# 실측 TCP 모델: 기본값, 계획만 수행하고 로봇은 움직이지 않음
ros2 run e0509_gripper_description curobo_planner_node.py

# 실측 TCP 모델 실제 실행: plan-only 결과 검토 후 저속 단일 target에서만 사용
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p measured_tcp_plan_only:=false

# 기존 SW 성공 baseline으로 즉시 복귀
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p tool_model_profile:=legacy_160mm
```

실측 프로필은 `grasp_tcp_link`를 플랜지 TOOL `+Z` 방향 `260mm`에 정의하고,
cuRobo `ee_link`로 사용한다. 따라서 planner의 수동 길이 보정은 `0mm`이며
기본 extra advance와 맨 왼쪽 전용 X 보정도 비활성화된다. 기본
`measured_tcp_plan_only=true`에서는 cuRobo 계획만 만들고 실제 모션 명령은
전송하지 않는다.

검출 Y가 보드 표면보다 크면 기존과 동일하게 `672mm`로 clamp한다. 보드 뒤쪽
target을 그대로 실행하는 것은 안전하지 않기 때문이다. 실측 TCP 프로필은
J4 한계에 걸리지 않는 접근 자세를 찾기 위해 pitch 후보를 `+15deg`까지
확장하지만, 첫 검증은 반드시 plan-only와 육안 확인을 거친다.

2026-06-11 plan-only 검증에서 실측 TCP의 `60mm` pre-approach는 반복적으로
계획됐지만, 보드 근처 `30~60mm` endpoint는 모든 자세에서 IK 실패했다. 따라서
실측 프로필은 긴 이동과 pre-approach까지 cuRobo로 검증하고, 마지막 `30mm`는
기존 성공 baseline과 같은 Doosan TOOL `+Z` MoveLine으로 직선 진입한다.
추가 진입은 없으며 TCP는 모델 보드 표면에서 최소 `30mm` stand-off를 유지한다.

## 2026-06-11 전환 및 트러블슈팅 기록

### 변경한 내용

- `grasp_tcp_link`를 `gripper_rh_p12_rn_base`의 TOOL `+Z 260mm` 자식 링크로
  추가했다.
- 새 cuRobo 프로필 `e0509_gripper_measured_tcp.yml`의 `ee_link`를
  `grasp_tcp_link`로 변경했다.
- 기존 `160mm` 수동 길이 보정, 기본 `65mm extra advance`, 맨 왼쪽 전용
  `+X 5mm` 보정은 실측 프로필에서 비활성화했다.
- 연장 파츠 근사 collision sphere를 `grasp_tcp_link`에 추가했다.
- 기존 성공 경로는 `tool_model_profile:=legacy_160mm`로 보존했다.
- 새 실측 프로필은 기본 `measured_tcp_plan_only:=true`로 실제 모션을 막았다.

### 첫 번째 실측 프로필 실행

run:

```text
logs/runtime/2026-06-11/
curobo_planner_node_20260611T151527-113f18f7.jsonl
```

관찰 결과:

```text
target: base_link (-105, 672, 618)mm
60mm pre-approach: 계획 성공
보드 근처 최종 endpoint: 전부 IK_FAIL
일부 자세: J4 약 358deg spline jump로 실행 전 거부
결과: 실제 모션 없이 안전 중단
```

### 두 번째 plan-only 실행

run:

```text
logs/runtime/2026-06-11/
curobo_planner_node_20260611T152431-967f636c.jsonl
```

pitch 후보를 `+15deg`까지 확장했지만 결과는 동일했다.

- `Y=612~614mm`의 60mm pre-approach는 여러 자세에서 계획 성공
- `Y=642mm`보다 깊은 최종 endpoint는 모든 자세에서 IK 실패
- 단순 자세 후보 부족이 아니라 보드 근처 최종 구간 계획 문제로 판단

### 현재 수정된 접근 정책

실측 TCP 프로필은 실패하는 최종 endpoint 계획을 반복하지 않는다.

```text
현재 scan pose
 -> cuRobo: 실제 grasp TCP 기준 60mm pre-approach 계획
 -> Doosan MoveSplineJoint: pre-approach까지 이동
 -> Doosan MoveLine TOOL +Z: guarded 30mm 직선 진입
 -> gripper close
 -> BASE -Z detach pull
 -> Doosan MoveLine TOOL -Z: 진입한 30mm 직선 역진
 -> scan pose 복귀
```

현재 상태는 **코드/빌드 검증 완료, 수정 후 plan-only 재검증 대기**다. 아직
실측 TCP 실제 실행 성공이나 수확 성공으로 기록하지 않는다.

### 세 번째 plan-only 실행 및 홈 복귀 원인

run:

```text
logs/runtime/2026-06-11/
curobo_planner_node_20260611T153321-6fdb516e.jsonl
```

관찰 결과:

- target: `raw=(-105,672,587)mm`, `grasp=(-105,672,617)mm`
- 앞의 3개 orientation 후보는 J4 약 `358deg` spline jump로 실행 전 거부
- 다음 orientation 후보에서 60mm pre-approach 계획 성공
- 성공 pre-approach goal: `(-105,612,612)mm`
- guarded final MoveLine `30mm` 준비 성공
- `measured_tcp_plan_only=true`이므로 실제 로봇 모션은 실행하지 않음

따라서 이 run은 계획 실패가 아니라 **실측 TCP plan-only 계획 성공**이다.

계획 직후 로봇이 홈으로 돌아간 원인은 plan-only 분기가
`/dsr01/curobo/pick_complete`를 발행했기 때문이다. 상위 scan executor가 이
신호를 실제 pick 종료로 해석하여 다음 단계 또는 홈 복귀를 실행했다.

수정 후 plan-only 분기는:

- `/pick_complete`를 발행하지 않는다.
- `measured_tcp_plan_only_hold` 이벤트를 JSONL에 기록한다.
- 상위 scan executor가 자동 진행하거나 홈으로 복귀하지 않도록 대기한다.

## 실기 검증 기준

- 첫 검증은 clear-space, 단일 target, 저속, E-stop 준비 상태에서 수행한다.
- flange, part tip, grasp center 위치를 각각 사진과 자로 남긴다.
- TOOL `+Z` 10mm 명령 시 실제 파지 홈이 같은 방향으로 10mm 이동하는지 확인한다.
- RViz/collision sphere와 실물 파츠 끝단 오차를 측정한다.
- 기존 SW baseline과 새 TCP 모델의 target 도달 오차를 비교한다.

## 다음 확인 조건

다음 plan-only run에서 아래 로그가 나와야 실제 실행 검토 단계로 넘어간다.

```text
MEASURED_TCP_PLAN_ONLY: valid pre-approach found and guarded 30mm final MoveLine prepared
MEASURED_TCP_PLAN_ONLY_HOLD: /pick_complete was not published
```

그 전에는 `measured_tcp_plan_only:=false`를 사용하지 않는다. 로그가 확인되어도
첫 실제 실행은 단일 SW target, 저속, clear-space, E-stop 준비 상태로 제한한다.
