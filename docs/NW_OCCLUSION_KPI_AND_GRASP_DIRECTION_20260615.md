# NW 잎/줄기 가림 수확 및 KPI 방향 - 2026-06-15

## 1. Place 검증 중단 지점

Slot5 row2 place는 release plan 자체는 생성됐지만, Above에서 release까지의
cuRobo 경로가 목표 수직선에서 `100.8mm` 벗어났다.

```text
ROW2_DESCENT_LINE_CHECK max_deviation=100.8mm limit=20.0mm
TAUGHT_TRAY_SLOT5_PLACE_BLOCKED
```

이는 실패가 아니라 위험한 곡선 하강을 실행 전에 차단한 정상 safety 결과다.
`20mm` 한계를 완화하여 강제 실행하지 않고, row2 place는 Cartesian constraint
또는 충돌 형상 보강 전까지 중단한다.

## 2. 다음 실험 범위

다음 대상은 큰 셀 `root/nw`이며, 잎과 줄기에 가려진 딸기를 대상으로 한다.

우선 순서:

1. NW scan pose에서 target discovery와 줄기 keypoint 가시성을 확인한다.
2. 줄기/KP1이 안정적으로 보이는 target만 기존 수평 진입 파이프라인으로 시도한다.
3. 줄기가 보이지 않거나 잎을 통과해야 하는 target은 강제 진입하지 않고
   `reobserve/skip` 대상으로 기록한다.
4. 이후 multi-view 관측과 일반 6-DoF grasp 후보 생성기를 비교한다.

## 3. KPI 자동 수집

실험 조건은 여러 시도 전에 한 번만 등록한다.

```bash
cd ~/doosan_ws/src/e0509_gripper_description
python3 scripts/set_experiment_context.py \
  --cell root/nw \
  --scene-id nw_leaf_stem_occlusion_v1 \
  --occlusion leaf_and_stem \
  --stem-shape mixed \
  --notes "NW 가림 조건 첫 반복 실험"
```

이후 생성되는 runtime JSONL에는 같은 `experiment_context`가 자동 첨부된다.

자동 KPI 요약:

```bash
python3 scripts/summarize_runtime_kpis.py --cell root/nw
```

자동 집계 항목:

- cuRobo 후보 계획 통과율과 계획 지연시간
- plan fail/reject 수
- 자동 파지 판정 가능률
- 접촉 후보 및 빈 파지 감지율
- Pick 시퀀스 시간과 종료 이벤트

## 4. 사람이 직접 확인해야 하는 최소 항목

그리퍼 position/current는 집게 사이의 접촉 가능성을 알려주지만, 줄기와 잎을
구분하지 못한다. 따라서 실제 줄기 파지, 분리, 후퇴 유지, 비목표 접촉은 초기
검증 표본에 사람 또는 영상 라벨이 필요하다.

개입 최소화 원칙:

- 개발 중 모든 run을 라벨링하지 않는다.
- 그리퍼 판정 임계값 보정용으로 빈 파지/줄기 파지/잎 접촉 각 10회만 우선
  라벨링한다.
- 이후에는 실패 run과 무작위 표본만 라벨링하여 자동 판정 precision/recall을
  지속 확인한다.
- 실제 정량 성공률 보고용 반복 실험에서는 모든 시도를 라벨링한다.

최신 시도 라벨:

```bash
python3 scripts/label_harvest_attempt.py
```

핵심 KPI 요약:

```bash
python3 scripts/summarize_harvest_kpis.py
```

## 5. 그리퍼 양방향 판정 현황

이미 구현된 실제 상태 인터페이스:

```text
/dsr01/gripper/read_state
/dsr01/gripper/present_position
/dsr01/gripper/present_current_raw
```

기존 `/dsr01/gripper/read_state` 경로는 `-1/-1`을 반환했지만,
2026-06-15 원본 `dsr_gripper_tcp`의 `/gripper_service/safe_grasp` 액션과
`/gripper_service/state`에서 position/current 양방향 판독을 확인했다.
수확 자동 판정은 원본 SafeGrasp 경로로 통합한다.

진단 명령:

```bash
ros2 service call /dsr01/gripper/read_state std_srvs/srv/Trigger "{}"
```

서비스가 정상 응답하면 조건별 상태 표본은 한 명령으로 자동 수집한다.

```bash
python3 scripts/collect_gripper_feedback.py --condition empty
python3 scripts/collect_gripper_feedback.py --condition stem
python3 scripts/collect_gripper_feedback.py --condition leaf_or_non_target
```

판정 의미:

- `GRASP_CONTACT_DETECTED`: position/current상 무언가 접촉한 후보
- `GRASP_EMPTY`: 집게가 거의 완전히 닫혀 빈 파지로 추정
- `GRASP_UNVERIFIED`: 상태 판독 실패 또는 임계값 근거 부족

`GRASP_CONTACT_DETECTED`는 실제 줄기 파지 성공과 동일하지 않다.

## 6. AnyGrasp / GraspGen 적용 방향

### AnyGrasp

AnyGrasp는 point cloud에서 빠르게 dense full-DoF grasp 후보를 생성하고,
objectness mask 및 collision detection을 지원한다. SDK는 라이선스 등록과
CUDA/MinkowskiEngine 환경이 필요하다.

### GraspGen

GraspGen은 diffusion 기반 6-DoF grasp 후보 생성과 discriminator scoring을
결합한 연구 프레임워크다. 일반 물체와 여러 gripper에 대한 후보 생성 성능을
목표로 한다.

### 프로젝트 판단

두 방법 모두 bent stem에 자동으로 맞는 줄기 파지를 보장하지 않는다. 현재
목표는 일반 물체를 안정적으로 집는 것이 아니라, 잎과 과실을 피하며 얇은
줄기의 지정 구간을 잡는 것이다.

따라서 다음 방식으로만 평가한다.

```text
NW multi-view RGB-D
 -> 줄기/과실/잎 ROI 또는 mask
 -> AnyGrasp 또는 GraspGen 후보 생성
 -> 줄기 근접도 + 접근 방향 + IK + collision + branch 필터
 -> 기존 KP1 rule baseline과 동일 KPI 비교
```

결론:

- 즉시 기존 KP1 파이프라인을 교체하지 않는다.
- AnyGrasp를 첫 6-DoF 후보 생성 baseline으로 검토한다.
- SDK 라이선스/환경 또는 줄기 후보 품질이 부적합하면 GraspGen/GraspNet 계열을
  offline baseline으로 비교한다.
- 가림으로 줄기 point cloud 자체가 없으면 grasp generator보다 multi-view
  reobserve가 먼저다.

참고 자료:

- AnyGrasp SDK: <https://github.com/graspnet/anygrasp_sdk>
- AnyGrasp paper: <https://arxiv.org/abs/2212.08333>
- GraspGen project: <https://graspgen.github.io/>
- GraspGen paper: <https://arxiv.org/abs/2507.13097>

## 7. 다음 실행 체크리스트

1. 원본 SafeGrasp로 빈 파지/줄기 파지/잎 접촉 보정 표본 수집
2. cuRobo close/verify 구간을 SafeGrasp action으로 통합
3. NW 실험 context 한 번 등록
4. `target_cell:=root/nw` 단일 셀 실행
5. 자동 runtime KPI 요약 및 필요한 표본만 사람 라벨
6. AnyGrasp는 실제 로봇 연결 전에 저장 point cloud로 offline 후보 품질 평가
