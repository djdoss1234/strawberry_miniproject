# Mini Project Experiment Results

작성 기준일: 2026-05-26

## Scope

이 문서는 저장소 안에서 확인 가능한 코드와 로컬 JSONL/image 로그를 기준으로 미니프로젝트의 결과를 정리한다. 외부에 저장된 시연 영상이나 사람이 별도로 작성한 실험 일지는 포함하지 않는다.

## Verified Runtime Capability

현재 소스에서 확인되는 end-to-end 흐름:

```text
YOLO strawberry detection
 -> HSV ripe filter
 -> RGB-D surface target estimation
 -> camera-to-base conversion
 -> cuRobo planning
 -> Doosan service execution
 -> soft gripper close
 -> taught egg-tray placement
```

핵심 근거 파일:

| Capability | Evidence |
| --- | --- |
| Detection/depth/transform/logging | `scripts/strawberry_yolo_node.py` |
| Planning/execution/place sequence | `scripts/curobo_planner_node.py` |
| Slot teaching | `scripts/teach_place_slots.py`, `config/place_slots.yaml` |
| Environment/collision setup | `config/environment.yaml`, `scripts/environment_visualizer.py` |
| Hardware gripper command | `src/gripper_service_node.cpp` |

## Log Summary

로컬 `logs/pick_attempts/`에 저장된 JSONL과 이미지 파일을 집계한 결과:

| Date | Attempt records | Sequence completion events | Mode distribution | Manual result labels |
| --- | ---: | ---: | --- | --- |
| 2026-05-18 | 45 | 27 | manual selected 15, manual locked 29, auto 1 | success 2, fail 3 |
| 2026-05-19 | 37 | 31 | manual selected 5, manual locked 21, auto 11 | none |
| 2026-05-21 | 86 | 53 | manual selected 14, manual locked 59, auto 13 | none |
| **Total** | **168** | **111** | **manual selected 34, manual locked 109, auto 25** | **success 2, fail 3** |

총 168장의 attempt 이미지가 로그와 함께 남아 있다.

## Correct Interpretation

현재 `/dsr01/curobo/pick_complete`는 planner/task sequence가 끝났음을 vision node에 알려주는 이벤트다. 이 이벤트는 센서로 확인된 grasp/place 성공을 뜻하지 않는다.

따라서 저장소 근거만으로 말할 수 있는 결과:

- 실제 로봇 실행 파이프라인이 통합되어 반복 attempt 로그를 남겼다.
- manual/auto target selection과 sequence completion 기록을 수집했다.
- 사람이 라벨한 일부 attempt에서 성공/실패 결과가 기록되어 있다.

현재 근거만으로 말하면 안 되는 결과:

- `111 / 168`을 수확 성공률이라고 표현하는 것
- 과실 무손상률 또는 충돌 안전성을 정량 달성했다고 표현하는 것
- 연속 3개 placement 성공을 원본 영상 없이 코드/로그만으로 증명했다고 표현하는 것

## Demonstration Evidence To Add

공개 포트폴리오를 보강하려면 다음 산출물을 추가하는 것이 좋다.

| Artifact | Content |
| --- | --- |
| Demo video link | 연속 pick-and-place 원본 또는 편집본 |
| Architecture image | Mermaid diagram 렌더링 또는 RViz/rqt graph screenshot |
| Before/after case | background depth 오차와 red-surface depth 개선 사례 |
| Planner failure case | IK/collision/world mismatch 분석 이미지 |
| Result table | 성공/실패 라벨을 충분히 수집한 후 산출한 KPI |

## Next Evaluation Protocol

다음 실전 프로젝트부터 각 attempt는 다음 결과 코드 중 하나를 기록한다.

```text
PERCEPTION_NO_TARGET
PERCEPTION_DEPTH_INVALID
TARGET_OUT_OF_WORKSPACE
IK_FAIL
START_STATE_COLLISION
PATH_MARGIN_REJECT
BRANCH_OR_SINGULARITY_REJECT
EXECUTION_REJECTED
GRASP_UNVERIFIED
GRASP_EMPTY
GRASP_DAMAGE
DETACH_FAIL
TRAY_NOT_FOUND
TRAY_POSE_UNCERTAIN
SLOT_OCCUPIED
PLACE_COLLISION
PLACE_DROP
SUCCESS
```

또한 run별로 commit hash, detector weight ID, calibration ID, scene ID, planner policy, collision object 목록, tray pose source, 영상/rosbag 경로를 함께 저장한다.

## 2026-06-08 SW Harvest And Marker Place Observation

실기에서 SW target 접근, gripper close, TOOL `-Z` 직선 retreat까지 반복 수행했다.
사용자 관찰 기준으로 한 시도는 잎에 밀려 파지하지 못했고, 다음 시도는 그리퍼가
잡았지만 딸기가 줄기에서 분리되지 않았다.

두 시도 모두 planner 로그상 접근/close/retreat는 완료되었으나, 실제 파지·분리
성공을 자동 검증하지 못했다. 이후 marker place는 tray localization age가 각각
`974s`, `1255s`로 허용치 `300s`를 초과하여 안전 차단되었다.

따라서 이번 결과는 수확 성공 또는 place 성공으로 집계하지 않는다. 확인된 성과는
다음과 같다.

- SW grasp target 접근과 직선 reverse retreat 반복 실행
- stale tray localization에 대한 place 안전 차단
- place 실패 후 후속 pick을 막는 persistent sequence hold latch 동작

다음 평가 전에는 `VERIFY_GRASP / VERIFY_DETACH` 결과 코드를 구현하고, fresh tray
localization을 사용한 단일 release 승인 실험을 수행해야 한다.

## 2026-06-09 Leftmost Grasp Path Verification

맨 왼쪽 과실의 파지 관찰 실행
`20260609T103947-913da046`을 runtime JSONL로 재검증했다.

| 항목 | 검증 결과 |
| --- | --- |
| top-down 선택 | target `x=-354.9mm`로 분기 진입 |
| top-down 계획 | `pre_approach_plan_failed`, 실제 모션 전 fallback |
| 실제 접근 정책 | horizontal fallback, target X `+10mm` 보정 |
| 실제 grasp variant | base X-axis `-5deg` |
| 실제 approach direction | `[0, 0.9962, -0.0872]` |
| 파지 관찰 | 사용자 실기 관찰상 잘 잡음 |
| 자동 파지 검증 | `GRASP_UNVERIFIED`, hardware position read 실패 |
| detach 검증 | `DETACH_UNVERIFIED` |

결론:

- **해결된 문제:** 맨 왼쪽 과실에 접근하지 못하던 문제는 top-down 계획 실패를
  실제 실행 전에 감지하고 수평 `-5deg` fallback으로 전환하여, 사용자 관찰상
  파지 가능한 상태가 되었다.
- **해결되지 않은 문제:** top-down 파지 자체는 아직 실행 성공 사례가 없다.
- 센서 기반 성공 근거가 없으므로 정량 수확 성공률에는 포함하지 않는다.
