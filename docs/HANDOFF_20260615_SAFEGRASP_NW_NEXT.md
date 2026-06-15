# HANDOFF - SafeGrasp 검증 완료, NW 가림 셀 시작 전

기준 시각: 2026-06-15
기준 커밋: `5f949aa docs: confirm original SafeGrasp TCP bridge`

## 1. 지금 상태

### 실기 검증 완료

- SW 단일 딸기에서 검출, 수평 접근, 줄기 파지, 아래 방향 분리, 후퇴 성공 사례 확보
- 실측 TCP 모델 적용: flange에서 실제 파지 중심까지 약 `260mm`
- 파지 모션 변경:
  - 그리퍼 position `600`으로 열린 상태 유지
  - 줄기 위쪽으로 수평 진입
  - 열린 상태로 BASE `-Z 30mm` 하강하여 KP1 부근에서 close
  - BASE `-Z 40mm` detach pull
  - TOOL `-Z` retreat
- 꺾인 줄기는 KP0/KP1의 국소 방향과 midpoint를 사용하도록 target 보정
- 긴 이동은 cuRobo 계획 + MoveSplineJoint, 접촉 구간은 Doosan MoveLine으로 분리
- operation speed `100%` 강제 및 후보/대기시간 축소 코드 반영
- 계란판 Slot0, Slot1, Slot3, Slot4 place 실기 성공
- Slot2는 30도 tilt 방식으로 도달 관찰했으나 약 3cm 오차가 남음
- Slot5 row2 하강은 수직선에서 `100.8mm` 이탈하여 release 전에 안전 차단
- NW 실험 context, runtime KPI 집계, 수동 라벨, PNG/JSON/Markdown KPI 보고서 도구 구현
- 원본 `Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge`의 SafeGrasp 실기 동작 확인

### 오늘 SafeGrasp에서 확인한 사실

기존 `/dsr01/gripper/read_state` 직접 경로는 `-1/-1`이었지만, 원본
`dsr_gripper_tcp`의 DRL TCP bridge 경로는 정상 동작한다.

```text
Gripper service node ready at 20.0 Hz
state: ready=true, present_position=700, present_current=8
empty SafeGrasp result: target reached without grasp
```

빈 파지 실행 중 position/current feedback가 연속 기록됐으며,
`grasp_detected=false`를 정상 반환했다.

자동 로그:

```text
logs/gripper_calibration/2026-06-15/safe_grasp_trials.jsonl
```

## 2. 아직 실행하지 않은 것

다음 항목은 Codex가 제안했지만 **아직 실기 실행 또는 통합하지 않았다**.

- 줄기 파지 SafeGrasp 보정 시험
- 잎/비목표 접촉 SafeGrasp 보정 시험
- 조건별 임계값 확정
- `curobo_planner_node.py`의 기존 `close + /dsr01/gripper/read_state`를
  `/gripper_service/safe_grasp` 액션으로 교체
- SafeGrasp feedback/result를 cuRobo runtime JSONL에 연결
- NW 잎/줄기 가림 셀 실제 Pick
- AnyGrasp/GraspGen 설치 또는 point-cloud offline 평가

## 3. 원본 SafeGrasp 실행 시 주의

동시에 두 그리퍼 제어 노드를 실행하면 안 된다.

원본 패키지는 workspace의 별도 경로에 설치돼 있다.

```text
~/doosan_ws/src/dsr_gripper_tcp
~/doosan_ws/src/dsr_gripper_tcp_interfaces
```

원본 `dsr_gripper_tcp`는 검증 직후에는 ready였지만 인계 문서 작성 시점에는
`/gripper_service/state`가 더 이상 publish되지 않았다. Claude Code 시작 시
충돌 노드를 확인한 뒤 원본 패키지를 다시 실행해야 한다.

두 패키지는 workspace에 복사된 소스이며 현재 별도 `.git` 저장소는 아니다.
주 프로젝트 git에는 SafeGrasp 연동 스크립트와 검증 문서만 기록돼 있다.

유지:

```text
e0509_gripper_description bringup.launch.py
```

종료:

```text
/dsr01/gripper_service_node
safe_grasp_ros_adapter.py
```

원본 실행:

```bash
source ~/doosan_ws/install/setup.bash
ros2 launch dsr_gripper_tcp gripper_service_node.launch.py \
  controller_host:=110.120.1.66 \
  namespace:=dsr01 \
  stop_existing_drl:=true \
  initialize_on_start:=true \
  init_attempts:=10 \
  goal_current:=400
```

첫 `INITIALIZE status 3`만 보고 종료하지 않는다. TCP bridge 재연결 후
`Gripper service node ready`가 출력될 수 있으므로 ready 또는 전체 재시도
종료까지 기다린다.

상태 확인:

```bash
ros2 topic echo /gripper_service/state --once
ros2 action list -t | grep safe_grasp
```

## 4. Claude Code가 바로 할 일

### 우선 1 - SafeGrasp 조건별 보정

빈 파지, 줄기 파지, 잎/비목표 접촉을 각각 최소 5회 수행한다. 처음에는
`max_current=400`, `current_delta_threshold=120`을 기준으로 분포를 확인한다.

```bash
python3 scripts/run_safe_grasp_trial.py \
  --condition stem \
  --target-position 700 \
  --max-current 400 \
  --current-delta-threshold 120 \
  --notes "manual stem fixture calibration" \
  --execute
```

잎 시험은 `--condition leaf_or_non_target`, 빈 파지는 `--condition empty`를 쓴다.

### 우선 2 - cuRobo 시퀀스에 SafeGrasp 통합

현재 코드는 다음 구조다.

```text
/dsr01/gripper/close Trigger
 -> sleep
 -> /dsr01/gripper/read_state Trigger
 -> GRASP_CONTACT_DETECTED | GRASP_EMPTY | GRASP_UNVERIFIED
```

이를 다음 구조로 교체한다.

```text
/gripper_service/safe_grasp action
 -> feedback position/current/current_delta 자동 기록
 -> result grasp_detected/object_lost 기록
 -> GRASP_CONTACT_DETECTED | GRASP_EMPTY | GRASP_UNVERIFIED 변환
```

주의:

- `grasp_detected=true`는 무언가 잡힌 것이며, 줄기 파지 성공은 아니다.
- 기존 Trigger 경로는 fallback으로 남긴다.
- SafeGrasp 서버가 없으면 fail-closed 또는 기존 경로로 명시적 fallback한다.
- 통합 후 Python compile/build를 수행하되, 실기 자동 반복은 바로 켜지 않는다.

### 우선 3 - NW 예비 실험

SafeGrasp 통합 후 Place를 끄고 `root/nw`에서 5회 예비 실험한다.

```bash
python3 scripts/set_experiment_context.py \
  --cell root/nw \
  --scene-id nw_leaf_stem_occlusion_v1 \
  --occlusion leaf_and_stem \
  --stem-shape mixed
```

측정할 핵심:

- target 발견 여부 및 KP1 가시성
- 접근/계획 성공 여부
- SafeGrasp 접촉/빈 파지
- 실제 줄기 파지, 분리, 유지 여부
- 잎 또는 비목표 접촉
- 사람 개입 여부

## 5. KPI 자동/수동 구분

### 자동 기록

- plan success/fail/reject와 planning latency
- MoveSplineJoint/MoveLine 실행 결과
- pick sequence time과 hold/recovery 원인
- SafeGrasp position/current/current_delta
- 접촉 후보, 빈 파지, object-lost

### 사람 또는 영상 라벨이 필요한 항목

- 실제 줄기를 잡았는지
- 딸기가 줄기에서 분리됐는지
- retreat 후 유지됐는지
- 잎/다른 딸기/구조물에 접촉했는지
- 목표 slot에 정상 배치됐는지

`grasp_detected=true`만으로 최종 수확 성공을 선언하지 않는다.

## 6. Place 현재 결론

현재 place는 marker localization이 아니라 Slot0/1/3 티칭값에서 계산한 고정
격자 baseline이다.

```text
Slot0 Slot3 Slot6 Slot9 Slot12
Slot1 Slot4 Slot7 Slot10 Slot13
Slot2 Slot5 Slot8 Slot11 Slot14
```

- row0/1: BASE `-Z` 방식으로 Slot0/1/3/4 검증
- row2: J3 실측 한계와 수직 하강 문제가 있음
- Slot5: line deviation `100.8mm > 20mm`, 정상 안전 차단
- row2는 Cartesian constraint/waypoint IK 또는 collision geometry 보강 전까지 중단

## 7. 보존 및 금지

- `scripts/측정.py`는 사용자 원본이다. 수정, stage, commit 금지.
- 기존 SW 동작 baseline을 전면 재작성하지 않는다.
- SafeGrasp 통합 전후 결과를 별도 로그로 비교한다.
- AnyGrasp/GraspGen은 기존 KP1 rule을 즉시 대체하지 않고 offline baseline부터 평가한다.

## 8. 관련 파일

```text
scripts/curobo_planner_node.py
scripts/run_safe_grasp_trial.py
scripts/set_experiment_context.py
scripts/summarize_runtime_kpis.py
scripts/generate_harvest_kpi_report.py
docs/SAFE_GRASP_STANDALONE_TEST_20260615.md
docs/GRIPPER_BIDIRECTIONAL_DIAGNOSIS_20260615.md
docs/HANDOFF_20260614_PLACE_TRAY_GRID.md
docs/NW_OCCLUSION_KPI_AND_GRASP_DIRECTION_20260615.md
docs/HARVEST_EXPERIMENT_OPERATION_PLAN_20260615.md
```

현재 git에서 사용자 원본 `scripts/측정.py`만 untracked 상태로 남아 있어야 한다.
