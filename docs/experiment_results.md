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
