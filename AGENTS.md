# AGENTS.md - Strawberry Harvest Robot Project Guide

## 1. Purpose

이 저장소는 Doosan E0509 로봇팔, ROBOTIS RH-P12-RN-A 그리퍼, Intel RealSense eye-in-hand RGB-D 카메라를 이용한 딸기 수확 로봇 프로젝트를 위한 ROS 2 패키지다.

현재 목표는 흰 보드에 부착한 모형 딸기 pick-and-place 데모에서 출발하여, 실제에 가까운 딸기 모형과 잎/줄기/가림/이동 가능한 tray를 포함한 환경에서 안전성과 성능을 정량 평가할 수 있는 농장형 수확 시스템으로 고도화하는 것이다.

에이전트는 단순히 동작하는 데모를 만드는 데서 멈추지 말고, 다음 질문에 답할 수 있는 시스템을 구축해야 한다.

- 어떤 딸기를 왜 선택했는가?
- target 좌표가 얼마나 정확한가?
- 경로가 왜 성공하거나 실패했는가?
- 실제 파지와 배치가 성공했는가?
- 계란판/tray가 이동해도 위치와 빈 slot을 다시 인식할 수 있는가?
- 과실 손상과 충돌 위험을 어떻게 평가했는가?
- 개선 전후가 수치로 얼마나 달라졌는가?

## 2. Current Baseline

현재 기준 파이프라인은 다음과 같다.

```text
RealSense RGB + aligned depth
 -> YOLO ripe/unripe strawberry detection
 -> HSV ripe safety filter
 -> red-surface depth estimation
 -> eye-in-hand calibration + E0509 FK
 -> /dsr01/curobo/pick_pose
 -> cuRobo approach/grasp/retreat/transfer planning
 -> Doosan MoveSplineJoint / MoveJoint execution
 -> RH-P12-RN-A soft close and release
 -> taught egg-tray slot placement
```

핵심 파일:

| 경로 | 역할 |
| --- | --- |
| `scripts/strawberry_yolo_node.py` | YOLO, depth target, camera-to-base transform, tracking, pick attempt logging |
| `scripts/curobo_planner_node.py` | cuRobo planning, pick/place sequence, Doosan execution, collision diagnostics |
| `scripts/pick_place_node.py` | cuRobo 없는 Doosan native motion baseline |
| `scripts/teach_place_slots.py` | egg-tray `above`/`release` pose teaching |
| `scripts/joint_jog_control.py` | 실기 pose 조정 도구 |
| `scripts/environment_visualizer.py` | RViz/MoveIt collision object 및 marker 시각화 |
| `src/gripper_service_node.cpp` | 실제 그리퍼 Modbus RTU 실행 계층 |
| `config/environment.yaml` | wall/tray 환경 모델 |
| `config/place_slots.yaml` | 티칭된 place slot |
| `config/regions.yaml` | coarse scan/quadtree 계획 스캐폴드 |
| `docs/system_architecture.md` | 현재 런타임 구조도 |
| `docs/project_retrospective_portfolio_roadmap.md` | 회고, 증거, 연구/포트폴리오 로드맵 |

## 3. Facts That Must Not Be Overstated

현재 저장소 기준으로 다음을 구분한다.

### Implemented

- RealSense + YOLO 기반 딸기 후보 검출
- ripe/unripe class 및 HSV 기반 pick 후보 필터
- bbox 내부 red pixel 기반 표면 depth 추정
- eye-in-hand calibration과 joint FK 기반 `base_link` target 변환
- cuRobo 기반 approach/grasp/retreat 및 일부 transfer planning
- Doosan `MoveSplineJoint`/`MoveJoint` 기반 실기 실행 연결
- gripper position command 기반 soft close
- egg-tray slot0~2의 `above`/`release` pose 저장
- whiteboard collision 모델 및 collision diagnostic
- pick attempt 이미지/JSONL logging

### Partially Implemented Or Demo-Tuned

- 현재 실행은 전 구간 cuRobo가 아닌 hybrid 구조다. 일부 place/home 구간은 `MoveJoint`를 사용한다.
- collision model은 demo 안정화를 위한 단계적 상태다. self-collision, table, tray body, placed-fruit obstacle 일부가 비활성화되어 있다.
- `/dsr01/gripper/stroke`는 현재 실제 force/contact success sensor로 신뢰하지 않는다.
- MoveIt은 현재 주 수확 planner가 아니라 환경 시각화 및 향후 baseline/scene 관리 후보다.

### Planned, Not Completed

- VLA 기반 판단/제어 연동
- 실행되는 quadtree scan manager
- 자동 tray pose localization
- 실제 농장 또는 실제 과실 검증
- force/current/tactile 기반 파지 성공 및 손상 판정
- 생산 수준의 완전한 collision/safety 검증

### Experiment Evidence

`logs/pick_attempts/`에는 2026-05-18, 2026-05-19, 2026-05-21 실험 기록이 있으며, 현재 확인된 수치는 다음과 같다.

- target attempt: 168건
- `pick_complete` event: 111건
- 수동 라벨: `success` 2건, `fail` 3건

`pick_complete`는 수확 성공률이 아니라 시퀀스 종료 이벤트다. 성공률을 보고하려면 상세 result code와 충분한 수동/자동 검증 라벨을 새로 수집해야 한다.

## 4. Target Project

후속 실전 프로젝트의 목표는 다음과 같다.

> 실제형 딸기 모형과 복잡한 재배 환경에서, perception uncertainty와 작업별 safety constraint를 반영하고, 여러 motion planning 정책의 한계를 정량 비교하여 안전하고 설명 가능한 수확 행동을 수행하는 로봇 시스템.

환경은 최소한 다음을 포함하도록 확장한다.

- 크기, 숙도 색상, 반사가 다른 딸기 모형
- 줄기와 꼭지
- 유연한 잎과 부분 가림
- 가까이 붙은 과실 cluster
- 흔들림 또는 compliance가 있는 부착 구조
- 움직일 수 있는 basket/tray와 occupied slot
- AprilTag/ArUco 부착 tray 및 marker가 일부 가려지는 조건

최종적으로 측정할 시스템 KPI:

| 영역 | KPI |
| --- | --- |
| Perception | detection precision/recall, maturity error, depth valid rate, 3D target error |
| Planning | plan success, IK failure distribution, planning latency, path length, minimum clearance |
| Execution | controller failure, cycle time, actual path deviation |
| Harvesting | detach success, grasp success, place success, drop rate, fruit damage score |
| Tray Localization | tray pose error, slot center error, occupied-slot recognition accuracy, relocalization success |
| Operation | recovery count, human intervention rate, area coverage efficiency |

## 5. Architecture Direction

현재 두 대형 노드에 기능을 계속 덧붙이지 않는다. 현재 데모는 재현 가능한 baseline으로 보존하고, 새 기능은 다음 책임 분리를 목표로 점진적으로 옮긴다.

```text
harvest_interfaces/
  TargetCandidate, HarvestResult, HarvestTarget.action

perception/
  detector, maturity_filter, depth_surface_estimator, target_tracker

calibration/
  transform_provider, calibration_validator

scene/
  scene_manager, tray_localizer, slot_occupancy_estimator, collision_world_bridge

planning/
  planner_adapter, curobo_adapter, moveit_adapter, native_baseline

task/
  harvest_state_machine, retry_policy, slot_manager, scan_manager

intelligence/
  vla_supervisor

evaluation/
  run_logger, benchmark_runner, metrics_report
```

우선 구현해야 할 계약은 `HarvestResult` 또는 그에 준하는 결과 기록 구조다. planner 개선이나 VLA 추가보다 먼저 성공, 실패, 중단 원인을 구분하여 측정 가능하게 만든다.

권장 state machine:

```text
IDLE
 -> SCAN
 -> SELECT_TARGET
 -> VALIDATE_TARGET
 -> PLAN_APPROACH
 -> APPROACH
 -> GRASP
 -> VERIFY_GRASP
 -> RETREAT
 -> PLAN_PLACE
 -> PLACE
 -> VERIFY_PLACE
 -> UPDATE_MAP
 -> SCAN or DONE
 -> RECOVER on failure
```

## 6. Planning Strategy

MoveIt, cuRobo, cuMotion 또는 vendor native motion을 종교적으로 선택하지 않는다. 각 도구의 역할과 한계를 동일 조건에서 비교한다.

정확한 관점:

- MoveIt은 robot model, planning scene, planner plugin, baseline planning 및 시각화에 활용 가능하다.
- cuRobo는 현재 GPU 기반 collision-aware motion generation 엔진으로 사용 중이다.
- NVIDIA cuMotion/Isaac ROS cuMotion은 향후 MoveIt 2 plugin 방식의 제품화 경로로 검토한다.
- Doosan native motion은 단순 구간 및 baseline 비교를 위해 유지한다.
- 프로젝트의 독자성은 프레임워크를 재작성하는 데 있지 않고, 딸기 수확에 필요한 scene, uncertainty, task constraint, failure recovery, evaluation을 설계하는 데 있다.

Planner comparison baseline:

| ID | 정책 | 용도 |
| --- | --- | --- |
| B0 | Doosan MoveJointX/MoveLine native path | 최소 baseline |
| B1 | Current cuRobo demo settings | 현재 재현 기준 |
| B2 | Strict cuRobo with richer collision world | 안전 모델 강화 비교 |
| B3 | MoveIt OMPL/Pilz baseline | framework 비교 |
| B4 | Custom hybrid task policy | 최종 작업별 개선안 |

각 baseline은 동일 target, 동일 obstacle definition, 동일 robot/tool pose에서 평가한다.

## 7. Quadtree And VLA Roles

### Quadtree

Quadtree는 motion planner가 아니라 작업 영역 관측 및 상태 관리 계층이다.

예상 cell 상태:

```text
UNSEEN
EMPTY
RIPE_CANDIDATE
UNRIPE
OCCLUDED
FAILED
DONE
```

구현 순서:

1. `config/regions.yaml`의 coarse 2x2 view pose를 실제로 티칭하거나 자동 생성한다.
2. scan manager가 영역별 발견/실패/완료 상태를 저장하도록 한다.
3. 불확실하거나 밀집된 영역에만 quadtree subdivision을 적용한다.
4. 중복 관측 감소, target discovery recall, cycle time을 비교한다.

### VLA

VLA는 초기에는 low-level motion executor가 아니라 semantic supervisor로 사용한다.

초기 역할:

- pickable/unripe/occluded/damaged/reobserve 판단
- 잎 또는 줄기 때문에 현재 접근이 부적절한 target veto
- 실패 이미지의 원인 태깅 보조
- quadtree 재관측 우선순위 보조

초기 금지 역할:

- raw trajectory 직접 실행
- deterministic collision checking 대체
- 센서 근거 없이 grasp success 선언

VLA 판단이 잘못되어도 planning/safety layer가 위험 실행을 거부할 수 있는 구조를 유지한다.

## 8. Phased Roadmap

### Phase 0 - Baseline Freeze And Measurement

- 현재 시연 가능한 코드를 branch/tag와 config snapshot으로 고정한다.
- run metadata에 commit, calibration ID, model ID, parameter profile, scene ID를 저장한다.
- result/failure taxonomy와 라벨링 방식을 구현한다.
- 기존 보드 데모에서 최소 30회 이상 결과 라벨을 확보한다.

### Phase 1 - Realistic Mock Cell

- 실제형 딸기/잎/줄기/cluster/tray 환경을 구축한다.
- reference fixture 또는 marker로 3D target error를 측정한다.
- 계란판에는 우선 AprilTag/ArUco를 부착하여 tray frame을 추정하고, frame 기준으로 slot center와 `above`/`release` pose를 자동 생성한다.
- marker 기반 slot 목표를 실제 hole center와 비교해 tray pose error 및 slot center error를 측정한다.
- fruit damage/drop/detach/place 평가 기준을 정의한다.
- collision sphere와 world model을 다시 검증한다.

### Phase 2 - Modular Runtime

- perception, planning, execution, task logic, logging을 분리한다.
- state machine과 recovery policy를 도입한다.
- ROS topic/service/action contract를 정리한다.
- rqt graph 및 rosbag/run artifact 보관 규칙을 만든다.

### Phase 3 - Planner Benchmark And Improvement

- B0~B3 baseline을 동일 benchmark scene에서 비교한다.
- 이동/회전된 계란판을 재인식해 생성한 place target을 planner 공통 입력으로 사용한다.
- 실패 taxonomy로 실제 병목을 찾는다.
- 필요한 부분에만 orientation sampling, preferred branch, clearance cost, scene update, retry policy를 추가한다.

### Phase 4 - Quadtree Scan

- coarse region scan을 우선 완성한다.
- subdivision과 persistent cell state를 추가한다.
- 탐색 효율과 수확 coverage를 측정한다.

### Phase 5 - VLA Supervisor

- 사람 라벨이 있는 semantic evaluation set을 만든다.
- rule-only, detector+rule, detector+VLA 판단을 비교한다.
- safety layer 바깥에서 우선 검증한 뒤 제한적으로 runtime에 연결한다.

## 9. Tray Localization And Automatic Placement

현재 `config/place_slots.yaml`의 slot0~2는 고정된 계란판을 대상으로 티칭한 pose다. 실전 프로젝트에서는 계란판이 조금만 이동하거나 회전해도 재티칭 없이 배치할 수 있어야 한다.

### Stage A - Fiducial Marker Baseline

- 계란판의 강체 부위에 AprilTag 또는 ArUco marker를 부착한다.
- RealSense로 marker pose를 추정하고 hand-eye transform을 통해 `base_link -> tray_frame`을 얻는다.
- tray 규격의 row, column, pitch, orientation을 config로 관리한다.
- `tray_frame` 기준 slot center, `above`, `release` pose를 자동 생성한다.
- 생성된 tray body, slot, 이미 채워진 과실 obstacle을 MoveIt scene 및 cuRobo world에 갱신한다.

marker baseline은 좌표계와 placement pipeline을 빠르고 정량적으로 검증할 수 있는 기준선이다.

### Stage B - Vision-Based Refinement

- RGB-D에서 계란판 외곽, hole pattern 또는 point cloud grid를 검출하여 marker pose를 보정한다.
- marker가 일부 가려졌거나 검출되지 않는 경우 tray geometry 기반 pose 추정을 fallback으로 사용한다.
- place 이후 RGB-D 이미지로 slot occupancy와 낙하/오배치를 판정한다.

### Stage C - Marker-Optional Operation

- marker 유무, 조명 변화, 부분 가림, tray 이동/회전 조건에서 pose 추정 성능을 비교한다.
- marker-only, marker+vision refinement, vision-only 방식의 pose error 및 place success를 기록한다.
- 현장 적용 방식은 마커 부착 가능성, 오염/가림 조건, 정확도 요구량에 따라 결정한다.

관련 result/failure code:

```text
TRAY_NOT_FOUND
TRAY_MARKER_LOST
TRAY_POSE_UNCERTAIN
SLOT_OCCUPIED
SLOT_GENERATION_FAIL
PLACE_TARGET_INVALID
```

## 10. Required Failure Taxonomy

새로운 런타임 및 실험 로그는 가능한 한 다음 failure/result code를 사용한다.

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
PLACE_COLLISION
PLACE_DROP
TRAY_NOT_FOUND
TRAY_POSE_UNCERTAIN
SLOT_OCCUPIED
SUCCESS
```

모든 pick attempt에 최소 다음 metadata를 기록한다.

```text
run_id
timestamp
git_commit
model_id
calibration_id
scene_id
tray_pose_source and tray_pose_error
target_id / region_id
raw_detection and transformed_target
planner_policy
active_collision_objects
plan_latency and execution_time
result_code
human_label or sensor_verification
associated_image/video/rosbag path
```

## 11. ROS Runtime Expectations

현재 주 인터페이스:

```text
/dsr01/joint_states
/dsr01/curobo/pick_pose
/dsr01/curobo/pick_complete
/dsr01/motion/move_spline_joint
/dsr01/motion/move_joint
/dsr01/gripper/open
/dsr01/gripper/position_cmd
/dsr01/gripper/stroke
```

변경 시 지켜야 할 원칙:

- `base_link`, camera, gripper/TCP frame convention을 명시하고 조용히 바꾸지 않는다.
- `pick_complete`를 성공 이벤트로 재사용하지 말고, 상세 결과 contract로 교체하거나 보완한다.
- MoveIt scene과 cuRobo world가 함께 존재하면 동일 환경 source 또는 명시적인 변환/검증 절차를 둔다.
- tray pose와 slot occupancy가 갱신되면 place target 생성기와 양쪽 collision world에 동일 source/timestamp로 반영한다.
- 실제 로봇 명령이 포함된 코드에는 motion busy, abort/recovery, service failure 상태를 기록한다.
- 자동 실행 전에 rqt graph와 활성 ROS interface를 확인할 수 있는 절차를 유지한다.

## 12. Engineering Rules For Agents

### Preserve The Baseline

- 기존 실기 동작을 깨뜨리는 전면 재작성은 하지 않는다.
- 현재 동작을 변경할 때는 어떤 benchmark 또는 검증으로 regression을 확인할지 먼저 정한다.
- 사용자가 만든 미커밋 파일, 실험 로그, calibration/model asset을 임의로 삭제하거나 되돌리지 않는다.
- calibration `.npz`, model weight, logs는 장비/실험 자산으로 취급한다.

### Keep Changes Modular

- `strawberry_yolo_node.py`와 `curobo_planner_node.py`에 새로운 기능을 무조건 계속 추가하지 않는다.
- 새 responsibility가 생기면 adapter, state machine, logger, scene manager 등 분리 가능한 경계를 먼저 검토한다.
- parameter를 새로 추가하면 코드 상수보다 YAML/ROS parameter profile을 우선 검토하고, 실험값의 의미와 단위를 문서화한다.

### Safety Before Demo Convenience

- 충돌 검사 비활성화나 safety margin 완화는 이유, 범위, 검증 조건을 기록하고 시연 편의 설정임을 분명히 남긴다.
- 실제 로봇에 새로운 trajectory 또는 자동 반복 실행을 붙이기 전에 low-speed/single-target/clear-space 검증 단계를 둔다.
- 파지 성공, 과실 손상, 충돌 회피를 센서 근거 없이 확정적으로 기록하지 않는다.
- 계란판 비전/marker 인식 기능을 추가할 때 고정 티칭 pose는 baseline 및 안전 fallback으로 유지한다.

### Verify Claims

- 구현 여부는 코드/설정에서 확인하고, 성공 결과는 로그/영상/정량 실험으로 확인한다.
- 포트폴리오나 자소서 문구를 작성할 때 `implemented`, `experimentally observed`, `planned`를 구분한다.
- 현재 로그만으로 정량 성공률을 주장하지 않는다.

### Test Proportionally

- Python 수정 후 관련 스크립트 `py_compile` 또는 적절한 unit test를 실행한다.
- YAML/world/slot 수정 후 loading 가능 여부와 marker/world 정합을 검증한다.
- planner/task 수정 후 offline planning 또는 virtual/simulation 단계를 먼저 확인하고, 실기 테스트 항목을 명시한다.
- 실제 로봇을 실행하지 못한 경우 최종 보고에 그 제한을 분명히 쓴다.

## 13. Documentation Rules

주요 문서 역할:

| 파일 | 유지 목적 |
| --- | --- |
| `AGENTS.md` | 향후 에이전트/협업자의 개발 방향과 행동 규칙 |
| `docs/system_architecture.md` | 현재 실행 구조도와 ROS 연결 |
| `docs/project_retrospective_portfolio_roadmap.md` | 사실 기반 회고, 포트폴리오, 면접, 연구 계획 |
| `README.md` | 재현 가능한 설치/실행 절차와 최신 주 파이프라인 |

새 기능이 주 실행 경로가 되면 최소한 다음을 업데이트한다.

- 실행 노드와 interface가 달라졌으면 architecture 문서
- phase, KPI, 구현/미구현 상태가 달라졌으면 roadmap 문서
- 설치/명령/필수 asset이 달라졌으면 README
- 앞으로의 작업 원칙이나 안전 기준이 달라졌으면 AGENTS.md

## 14. Portfolio Narrative

이 프로젝트의 서사는 다음을 중심으로 유지한다.

> 저는 화면에서 객체를 검출하는 데서 끝내지 않고, depth와 eye-in-hand 좌표 변환으로 실제 robot target을 만들고, motion planning과 controller execution, gripper, place pose까지 연결했습니다. 그 과정에서 detection이 성공해도 depth 오차, IK branch, collision world, 파지 품질 때문에 전체 작업은 실패할 수 있다는 점을 경험했고, 실패를 레이어별로 기록하고 검증하는 방향으로 시스템을 고도화하고 있습니다.

피해야 할 문장:

- "완전한 농장 자동 수확 시스템을 구현했다."
- "충돌 없는 안전 경로를 완성했다."
- "MoveIt/cuRobo는 현업에서 사용하지 않는다."
- "현재 로그로 높은 성공률을 달성했다."
- "VLA와 quadtree를 적용했다." (실제 구현 전)

사용 가능한 확장 목표 문장:

> 후속 프로젝트에서는 실제형 딸기 환경에서 AprilTag/ArUco 및 RGB-D 기반 계란판 pose 추정으로 배치 위치를 자동 생성하고, MoveIt planning scene과 GPU 기반 motion generation을 비교 평가하며, quadtree 기반 영역 탐색과 VLA 기반 semantic supervisor를 결합하여 안전하고 설명 가능한 수확 시스템으로 확장할 계획입니다.
