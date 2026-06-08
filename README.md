# Strawberry Harvest Robot Prototype

RealSense RGB-D 비전으로 딸기 후보의 3D 위치를 추정하고, cuRobo 기반 모션 계획과 Doosan E0509 실행 서비스를 연결하여 실제 pick-and-place를 수행하는 ROS 2 미니프로젝트 결과물입니다.

현재 저장소는 흰 보드에 배치한 모형 딸기와 계란판을 이용한 프로토타입을 정리한 것으로, 후속 프로젝트에서는 실제형 과실, 잎/줄기 가림, 이동 가능한 tray, quadtree 탐색, VLA 판단 계층까지 확장할 계획입니다.

## Demo Pipeline

```text
Intel RealSense RGB-D (eye-in-hand)
  -> YOLO ripe/unripe strawberry detection
  -> HSV ripe safety filter
  -> red-surface depth estimation
  -> hand-eye calibration + E0509 forward kinematics
  -> /dsr01/curobo/pick_pose
  -> cuRobo pre-approach/grasp endpoint validation
  -> stop at pre-approach
  -> Doosan MoveLine TOOL +Z low-speed final grasp advance
  -> gripper close -> Doosan MoveLine TOOL -Z straight reverse retreat
  -> Doosan MoveSplineJoint / MoveJoint hybrid execution
  -> RH-P12-RN-A soft close
  -> optional guarded ArUco-derived egg-tray slot placement
```

상세 architecture diagram과 sequence diagram은 [docs/system_architecture.md](docs/system_architecture.md)를 참고하세요.
단계별 planner 역할과 시뮬레이션 재생 JSONL 규격은
[docs/runtime_pipeline_and_simulation_logs.md](docs/runtime_pipeline_and_simulation_logs.md)를 참고하세요.

## Hardware And Stack

| Category | Component |
| --- | --- |
| Robot arm | Doosan E0509 |
| Gripper | ROBOTIS RH-P12-RN-A |
| Camera | Intel RealSense RGB-D, eye-in-hand mounting |
| Middleware | ROS 2 Humble |
| Perception | Ultralytics YOLO, OpenCV HSV filtering, aligned depth |
| Calibration | Eye-in-hand transform and robot FK |
| Planning | cuRobo MotionGen; MoveIt/RViz environment visualization |
| Execution | Doosan ROS 2 motion services and Tool Flange Serial / Modbus RTU gripper control |

## Implemented Features

- YOLO 기반 숙성/미숙 딸기 후보 검출과 HSV 보조 필터
- bbox 중심값 대신 빨간 표면 pixel depth를 우선하는 3D target 추정
- eye-in-hand calibration과 현재 joint FK를 이용한 `base_link` 좌표 변환
- target lock, EMA tracking, manual/auto pick publish, JSONL/image experiment logging
- cuRobo 기반 pre-approach, grasp endpoint 검증 및 안전 거리 확보 후 transfer 경로 계획
- pre-approach 정지 후 Doosan `MoveLine` TOOL `+Z` 저속 직선 파지 진입
- 파지 후 동일 거리를 Doosan `MoveLine` TOOL `-Z`로 역주행하여 안전하게 후퇴
- cuRobo trajectory를 Doosan `MoveSplineJoint` 실행 명령으로 연결
- 그리퍼 압상을 줄이기 위한 단계적 position soft close
- 계란판 slot0~2의 `above`/`release` pose teaching
- 최신 ArUco 15-slot 결과를 읽는 guarded marker-place sequence
- 실측 whiteboard collision world와 start collision diagnostic
- RViz/MoveIt `CollisionObject` 기반 환경 시각화
- cuRobo 없는 Doosan native motion baseline 노드

## Current Results

코드와 로컬 실험 로그에서 확인할 수 있는 범위의 결과입니다.

| Item | Confirmed Value |
| --- | ---: |
| Experiment dates | 2026-05-18, 2026-05-19, 2026-05-21 |
| Pick target attempts logged | 168 |
| `pick_complete` sequence events logged | 111 |
| Manually labeled `success` | 2 |
| Manually labeled `fail` | 3 |

`pick_complete`는 실제 수확 성공률이 아니라 시퀀스 종료 이벤트입니다. 현재 기록만으로 높은 정량 성공률을 주장하지 않으며, 후속 버전에서는 상세 `HarvestResult`와 충분한 결과 라벨을 수집합니다.

연속 계란판 배치 시연 영상이 포트폴리오에 함께 제시되는 경우에만 해당 영상으로 확인 가능한 성공 결과를 별도 기술합니다.

자세한 결과 해석과 향후 평가 방법은 [docs/experiment_results.md](docs/experiment_results.md), [docs/project_retrospective_portfolio_roadmap.md](docs/project_retrospective_portfolio_roadmap.md)를 참고하세요.

## Repository Layout

```text
.
|-- AGENTS.md                              # 후속 프로젝트 개발/안전/검증 원칙
|-- docs/
|   |-- system_architecture.md             # 현재 ROS pipeline diagrams
|   |-- experiment_results.md              # 확인 가능한 결과 및 제한
|   `-- project_retrospective_portfolio_roadmap.md
|-- config/
|   |-- curobo/                            # robot/collision sphere configuration
|   |-- environment.yaml                   # wall/tray world description
|   |-- place_slots.yaml                   # taught placement targets
|   `-- regions.yaml                       # future quadtree scan scaffold
|-- launch/
|-- scripts/
|   |-- strawberry_yolo_node.py            # perception and target generation
|   |-- curobo_planner_node.py             # planning and execution sequence
|   |-- pick_place_node.py                 # native motion baseline
|   |-- teach_place_slots.py
|   |-- joint_jog_control.py
|   `-- environment_visualizer.py
|-- src/                                   # C++ gripper/bridge nodes
`-- urdf/
```

## Main Runtime Interfaces

| Interface | Purpose |
| --- | --- |
| `/dsr01/joint_states` | current robot joint state |
| `/dsr01/curobo/pick_pose` | strawberry target in `base_link` |
| `/dsr01/motion/move_spline_joint` | execution of planned trajectory |
| `/dsr01/motion/move_line` | TOOL `+Z` 최종 직선 진입 및 TOOL `-Z` 동일 경로 후퇴 |
| `/dsr01/motion/move_joint` | fixed pose / demo short motion execution |
| `/dsr01/gripper/open` | open gripper |
| `/dsr01/gripper/position_cmd` | stepwise soft-close position command |
| `/dsr01/curobo/pick_complete` | current sequence completion notification |

## Build And Run

### Dependencies

- Ubuntu and ROS 2 Humble
- Doosan ROS 2 packages (`dsr_msgs2`, robot description/controller packages)
- RH-P12-RN-A description package
- Python: `numpy`, `scipy`, `opencv-python`, `pyrealsense2`, `ultralytics`, `pyyaml`, `torch`
- NVIDIA CUDA-compatible environment and cuRobo for GPU motion planning

### Build

```bash
cd ~/doosan_ws
colcon build --packages-select e0509_gripper_description --symlink-install
source install/setup.bash
```

### Real Robot Bringup

```bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<robot_ip>
```

### Planner And Strawberry Vision

```bash
ros2 run e0509_gripper_description curobo_planner_node.py
ros2 run e0509_gripper_description strawberry_yolo_node.py
```

Marker place는 기본 비활성화되어 있습니다. 먼저 최신 tray localization을 생성한 뒤,
release를 끈 preview mode로 slot above clearance를 확인합니다.

```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p execute_marker_place_release:=false
```

단일 slot above 검증 후에만 실제 release를 명시적으로 승인합니다.

```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p execute_marker_place_release:=true
```

또는 planner warm-up 이후 vision node를 시작하는 launch:

```bash
ros2 launch e0509_gripper_description curobo_vision.launch.py
```

### Teaching And Visualization

```bash
ros2 run e0509_gripper_description joint_jog_control.py
ros2 run e0509_gripper_description teach_place_slots.py
ros2 launch e0509_gripper_description environment_visualization.launch.py
```

## Local Assets Not Published

다음 파일은 장비 종속성, 개인정보/실험 이미지, 모델 배포 여부 및 저장소 용량을 고려해 기본적으로 GitHub 공개 커밋에서 제외합니다.

```text
models/*.pt                    # trained YOLO weights
config/*.npz                   # hand-eye calibration result
config/calibration/            # calibration artifacts
logs/                          # raw attempt images and JSONL records
```

실행 시에는 로컬에 다음 파일이 필요합니다.

```text
models/best.pt
config/calibration_eye_in_hand_1.npz
```

## Known Limitations

- Marker-derived place는 최신 localization JSON을 읽을 수 있지만, tray/table collision
  geometry가 아직 비활성화되어 있어 preview 및 저속 단일 slot 검증이 필요합니다.
- 현재 실행은 전 구간 cuRobo가 아닌 hybrid 방식이며, 짧은 place/home 동작에는 `MoveJoint`를 사용합니다.
- 2026-06-07 SW 실기에서 수평 정면 접근은 확인했지만 최종 진입 깊이가 부족해 실제 줄기 파지는 아직 성공하지 못했습니다.
- `grasp OK`와 `pick_complete`는 모션/시퀀스 완료 이벤트이며 실제 파지 성공 판정이 아닙니다.
- demo 안정화를 위해 self/table/tray/placed-fruit collision 검사 일부가 비활성화되어 있습니다.
- `/dsr01/gripper/stroke`는 현재 실제 파지 성공 또는 힘 피드백으로 사용할 수 없습니다.
- VLA와 실행되는 quadtree 탐색은 후속 프로젝트 계획입니다.

## Next Project Direction

1. `HarvestResult`와 failure taxonomy를 추가해 성공/실패 원인을 정량 기록합니다.
2. 실제형 딸기 모형, 줄기, 잎 가림, cluster 환경을 구성합니다.
3. AprilTag/ArUco 및 RGB-D로 `tray_frame`을 인식해 slot pose와 occupancy를 자동 생성합니다.
4. MoveIt Planning Scene, cuRobo/cuMotion, Doosan native motion을 동일 benchmark에서 비교합니다.
5. Quadtree는 작업 영역 탐색/상태 관리에, VLA는 수확 가능성 판단 supervisor에 적용합니다.

개발 원칙과 단계별 기준은 [AGENTS.md](AGENTS.md)에 기록되어 있습니다.

## Project Notes

이 저장소는 실제 로봇 통합 미니프로젝트 결과물을 포트폴리오 형태로 정리한 것입니다. 구현된 기능, 실험에서 관찰된 결과, 향후 계획을 구분하여 기록하며, 팀 프로젝트 산출물의 기여 범위는 원본 브랜치/커밋 이력과 함께 설명합니다.
