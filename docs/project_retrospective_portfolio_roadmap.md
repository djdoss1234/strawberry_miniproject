# 딸기 수확 로봇 프로젝트 회고, 포트폴리오, 실전 고도화 로드맵

작성 기준일: 2026-05-26
대상 패키지: `e0509_gripper_description`
목적: 포트폴리오, 자기소개서 재료, 면접 대비, 실제 딸기 모형/농장형 후속 프로젝트 설계

## 2026-06-07 수확 모션 업데이트

SW 단일 딸기 실험에서 기존 고정 orientation의 접근축이 약 `+14.7 deg` 위로
기울어져 그리퍼가 아래에서 위로 접근하는 원인을 확인했다. 완전 수평
쿼터니언 하나를 강제하면 SW 시작 관절 구성에서 IK가 실패했기 때문에,
base-frame X축 pitch 보정 후보를 `0.0 / +4.7 / +9.7 deg` elevation 순으로
탐색하도록 변경했다.

또한 cuRobo joint spline만으로 마지막 TCP 정면 진입을 보장하기 어려워,
cuRobo가 pre-approach 및 grasp endpoint의 IK/collision/branch 안전성을 검증하고,
로봇은 pre-approach에서 완전히 멈춘 뒤 Doosan `MoveLine`으로 TOOL `+Z`
방향을 저속 직선 진입하는 hybrid 정책을 적용했다.

2026-06-07 실기에서 수평 정면 접근 방향과 stop-then-straight 동작은 확인했지만,
명령한 `140mm` 진입 후에도 실제 줄기를 감쌀 만큼 깊이 들어가지 않아 파지는
실패했다. 따라서 현재 성과는 "정면 접근 방향 및 실행 구조 검증"이며,
"실제 줄기 파지 성공"은 아직 주장하지 않는다.

다음 작업은 남은 물리 거리를 측정하고 wall clearance를 확인한 뒤 최종 진입량을
`5~10mm` 단위로 증가시키는 것이다. 자세한 근거와 로그는
`docs/harvest_motion_session_20260607.md`에 기록했다.

## 0. 먼저 정확히 말해야 하는 것

이 문서는 현재 작업 폴더의 코드, 설정, 실험 로그, git 브랜치 이력과 사용자가 제공한 이전 대화 요약을 바탕으로 정리했다. 제공되지 않은 과거 대화 전문이나 저장소 밖의 시연 영상 내용까지 확인한 것은 아니다.

따라서 포트폴리오 문장은 다음처럼 증거 수준을 나누어 사용한다.

| 구분 | 말해도 되는 내용 | 근거 |
| --- | --- | --- |
| 구현 확인 | RealSense/YOLO 인식, depth 기반 3D target 생성, eye-in-hand 변환, cuRobo 계획, Doosan 실행, soft close, slot 티칭 구조를 구현했다 | 소스 코드와 설정 파일 |
| 실험 확인 | 2026-05-18/19/21에 target 전송 168건, `pick_complete` 이벤트 111건, attempt 이미지 168장이 저장되어 있다 | `logs/pick_attempts/` |
| 제한적 성공 증거 | 수동 결과 라벨에는 `success` 2건, `fail` 3건이 있다 | JSONL 로그 |
| 영상이 있을 때만 사용 | 정상 딸기 3개를 계란판 세 슬롯에 순차 배치하는 시연을 완료했다 | 별도의 원본 영상/발표 자료를 함께 제시할 때 |
| 아직 계획 단계 | VLA, quadtree 실제 탐색 노드, 농장 환경 검증, 자동 tray 인식, force 기반 파지 확인 | 코드에서 완성 구현 미확인 |

핵심은 결과를 축소하는 것이 아니라, 면접에서 바로 방어할 수 있는 말만 쓰는 것이다.

## 1. 프로젝트 한 줄 정의

**Intel RealSense eye-in-hand RGB-D 카메라와 YOLO로 딸기 후보의 3D 위치를 추정하고, cuRobo로 Doosan E0509의 접근/파지/후퇴/이송 경로를 계획하여 RH-P12-RN-A 그리퍼로 계란판 배치를 시도하는 실제 로봇 통합 프로토타입.**

포트폴리오 제목 후보:

1. `검출 좌표에서 실제 로봇 실행까지: RGB-D와 cuRobo 기반 딸기 수확 로봇`
2. `Doosan E0509 딸기 수확 프로토타입: Perception-to-Execution 통합과 실패 분석`
3. `실물 Pick & Place의 오차를 다루는 로봇 시스템: 딸기 수확 데모에서 농장형 시스템으로`

면접에서 먼저 말할 핵심:

> 단순 객체 검출 과제가 아니라, 검출된 딸기를 로봇 기준 3D 목표로 변환하고, 실제 하드웨어의 IK branch, 충돌 세계, 그리퍼 파지, place 자세 문제를 통합적으로 해결해 본 프로젝트입니다.

## 2. 현재 구현의 사실 지도

### 2.1 구현되어 있는 것

| 기능 | 구현 내용 | 핵심 파일 |
| --- | --- | --- |
| 로봇/그리퍼 모델 | Doosan E0509와 RH-P12-RN-A 결합 URDF/Xacro, 실제/가상/Gazebo bringup | `urdf/e0509_with_gripper.urdf.xacro`, `launch/bringup.launch.py` |
| 실제 그리퍼 통신 | Tool Flange Serial 위 Modbus RTU, open/close/position command 서비스/토픽 | `src/gripper_service_node.cpp`, `include/.../modbus_rtu.hpp` |
| 딸기 검출 | Ultralytics YOLO weight `models/best.pt` 로 bbox/class 추론 | `scripts/strawberry_yolo_node.py` |
| ripe 보조 필터 | red ratio, strong-red ratio, saturation threshold, unripe class 제외 | `scripts/strawberry_yolo_node.py` |
| 2D-to-3D | aligned depth에서 빨간 픽셀 우선 샘플링, 20 percentile near-surface depth | `scripts/strawberry_yolo_node.py` |
| 좌표 변환 | `T_base_gripper * T_gripper_camera * p_camera`, 조인트 기반 FK 포함 | `scripts/strawberry_yolo_node.py`, `config/calibration_eye_in_hand_1.npz` |
| target 선택/로그 | EMA tracking, lock, manual/auto publish, 이미지와 JSONL 저장 | `scripts/strawberry_yolo_node.py`, `logs/pick_attempts/` |
| 모션 계획 | cuRobo `MotionGen`: approach, grasp, retreat, fixed-joint transfer | `scripts/curobo_planner_node.py` |
| 계획 실행 | trajectory를 최대 12점으로 downsample 후 Doosan `MoveSplineJoint` 호출 | `scripts/curobo_planner_node.py` |
| 데모 시퀀스 | open, approach, grasp, soft close, retreat, place above/release, home | `scripts/curobo_planner_node.py` |
| 환경 모델 | 실측 whiteboard cuboid, RViz/MoveIt `CollisionObject` 시각화, cuRobo 공통 YAML 로딩 | `config/environment.yaml`, `scripts/environment_visualizer.py` |
| place 티칭 | slot별 `above`/`release` joint 및 TCP 저장; slot0~2에 값 존재 | `scripts/teach_place_slots.py`, `config/place_slots.yaml` |
| 조인트 미세 이동 | step 기반 `MoveJoint` 티칭 보조 | `scripts/joint_jog_control.py` |

### 2.2 구현 흔적은 있으나 현재 주 실행 경로가 아닌 것

| 기능 | 상태와 해석 |
| --- | --- |
| Grounding DINO 범용 물체 추적 | `object_tracking_node.py`와 기존 README에 남아 있지만 현재 `curobo_vision.launch.py`는 YOLO 딸기 노드를 실행한다. 이전 실험 계보로 설명해야 한다. |
| ArUco target tracking | `marker_tracking_node.py`가 존재하며 좌표계 검증/표식 기반 실험에 유용하다. 딸기 주 파이프라인은 아니다. |
| cuRobo 없는 Doosan 네이티브 접근 | `pick_place_node.py`가 MoveJointX/MoveLine 기반 대조 경로를 제공한다. baseline 실험으로 활용할 가치가 있다. |
| MoveIt 연결 | `environment_visualizer.py`가 `CollisionObject`를 발행하지만, 이 패키지 자체에 MoveIt planning으로 딸기 수확을 수행하는 주 경로는 확인되지 않는다. 현재 역할은 시각화/환경 검증 쪽이다. |

### 2.3 아직 완료되지 않은 것

| 항목 | 현재 상태 |
| --- | --- |
| Quadtree 탐색 실행 | `config/regions.yaml`에 `coarse_2x2_then_quadtree` 설계 스캐폴드만 존재하고 `quadtree.enabled: false`; 이를 사용하는 실행 노드는 확인되지 않는다. |
| VLA | 현재 패키지에서 VLA inference/action 연결 코드는 확인되지 않는다. |
| 실제 농장 검증 | 흰 보드/모형/계란판 데모 설정 중심이며 잎, 줄기, 조명 변화, 실제 과실 손상 평가 데이터는 없다. |
| 자동 성공 판정 | `/gripper/stroke`가 명령값 형태로 발행되므로 force/contact 검증으로 사용하지 않고 있다. |
| 완전한 충돌 모델 | table, tray body, placed strawberry obstacle, self-collision이 현 데모 설정에서 비활성화되어 있다. |

## 3. 시스템 구조와 작동 원리

### 3.1 하드웨어 및 실행 계층

| 계층 | 구성 | 역할 |
| --- | --- | --- |
| Sensing | Intel RealSense RGB-D, eye-in-hand | color/depth frame 획득 |
| Manipulation | Doosan E0509 6축 arm | TCP 이동 및 경로 실행 |
| End-effector | ROBOTIS RH-P12-RN-A | 딸기 몸통 soft close 파지 및 release |
| Fixture | whiteboard wall, 모형 딸기 | 수확 대상 배치 |
| Destination | egg tray | 티칭된 slot에 place |

그리퍼 노드는 실제 모드에서 Doosan tool flange serial 서비스에 연결하고 Modbus RTU 명령으로 목표 위치를 전송한다. 현재 `stroke` 토픽은 실제 접촉 힘이 아니라 명령 위치를 publish하는 구조이므로 파지 성공 센서로 해석하면 안 된다.

### 3.2 Perception 원리

실행 흐름:

```text
RealSense RGB + aligned depth
 -> YOLO bbox/class
 -> HSV 기반 ripe 후보 필터
 -> bbox 내부 red pixel 중심 depth 표면 추정
 -> camera intrinsic deprojection
 -> eye-in-hand transform + FK
 -> base_link 기준 PoseStamped publish
```

현재 depth estimator의 중요한 선택은 bbox 중앙 한 점이 아니라 다음 절차를 쓰는 것이다.

1. bbox 내부를 2 pixel 간격으로 샘플링한다.
2. 유효 depth 중 빨간 픽셀의 depth가 20개 이상이면 이를 우선한다.
3. depth 분포의 20 percentile을 표면 깊이로 선택한다.
4. 해당 깊이와 3 cm 이내의 점들에서 median pixel을 target pixel로 삼는다.

이 설계의 이유는 둥근 모형 딸기의 bbox 내부에 뒤쪽 흰 보드가 섞이면 중심점/전체 median이 배경 깊이를 고르는 문제가 있기 때문이다.

3D 변환은 다음과 같다.

```text
p_cam = deproject(u, v, depth, camera_intrinsic)
p_base = T_base_gripper(q_current) * T_gripper_camera * p_cam
```

`T_gripper_camera`는 `.npz` 캘리브레이션 파일에서 읽고, `T_base_gripper`는 현재 `/dsr01/joint_states`를 사용한 E0509 FK로 계산한다. 현재 로컬에 두 캘리브레이션 파일이 있으며 translation도 다르므로, 실전 프로젝트에서는 캘리브레이션 버전과 실험 run을 반드시 연결해야 한다.

### 3.3 Planning 원리

YOLO 노드는 딸기 중심에 가까운 `p_strawberry`를 publish하고, planner는 TCP/그리퍼 길이 및 벽 법선 방향을 고려하여 실제 end-effector 목표를 만든다.

```text
straw = perceived_target + [0, 0, GRASP_Z_BIAS]
ee_approach = straw - (APPROACH_OFFSET + GRIPPER_LEN) * WALL_UNIT
ee_grasp    = straw - (grasp_offset + GRIPPER_LEN) * WALL_UNIT
ee_retreat  = straw - (RETREAT_OFFSET + GRIPPER_LEN) * WALL_UNIT
```

현재 주요 값:

| 파라미터 | 현재 값 | 의미 |
| --- | ---: | --- |
| `APPROACH_OFFSET` | 0.15 m | 벽면 대상 전방 접근 여유 |
| `GRASP_RETRY_OFFSETS` | -0.05, -0.04, -0.03, 0.0 m | 파지 깊이 후보 |
| `RETREAT_OFFSET` | 0.36 m | place 이송 전에 충분히 빠짐 |
| `stem_grasp_offset_from_kp0_m` | +0.010 m | KP0에서 KP2 전체 줄기 방향으로 최대 10mm 이동한 파지 목표 |
| `grasp_target_base_z_trim_m` | +0.010 m | 줄기 방향 target 생성 후 물리적으로 base +Z 10mm 추가 |
| `GRASP_Z_BIAS` | 0.000 m | fusion 줄기 방향 보정과 중복되지 않도록 base Z 보정 비활성화 |
| `GRIPPER_LEN` | 0.160 m | ee link에서 TCP까지의 거리 |

모든 파지 orientation을 벽면 대응 자세로 고정하고, far-right 대상에는 깊은 IK retry를 생략한다. left target에는 home과 같은 safe transfer 자세를 통과하도록 하는 휴리스틱이 들어가 있다.

### 3.4 Collision/Execution 원리

현재 cuRobo world에는 `environment.yaml`에서 enabled인 cuboid만 들어간다. whiteboard wall만 활성화되어 있고 table 및 egg tray body는 비활성화되어 있다. planner 결과 trajectory는 운영 joint range를 벗어나면 거부되며, 통과하면 degree 단위로 바꿔 `MoveSplineJoint`에 전달된다.

데모에서 사용 중인 혼합 실행 정책:

| 이동 구간 | 실행 |
| --- | --- |
| approach/grasp/retreat, place above로 큰 이동 | cuRobo plan + `MoveSplineJoint` |
| place release, release 후 above retreat, home | `USE_MOVEJ_FOR_DEMO_PLACE=True`에 따라 `MoveJoint` |

따라서 현재 시스템을 “전 구간 cuRobo collision-aware 계획”이라고 설명하면 틀리고, “중요 접근/이송은 cuRobo를 쓰고 짧은 티칭 구간은 데모 안정성을 위해 네이티브 고정 자세 실행을 병용한 hybrid pipeline”이라고 설명해야 한다.

## 4. 진행 과정과 문제 해결 서사

### 단계 A. 범용 물체 추적에서 딸기 전용 인식으로

원본 `main`에는 Grounding DINO 기반 범용 물체 노드와 cuRobo 기본 pick sequence가 있다. 이후 YOLO 딸기 weight, ripe/unripe 판단, HSV 필터, attempt logging이 추가되어 작물 대상의 도메인 문제를 다루는 방향으로 전환되었다.

포트폴리오 포인트:

> 범용 검출 데모에서 끝내지 않고, 숙도 판단 오류와 실제 모형의 색/반사 특성을 처리하기 위해 작물 전용 detector와 rule-based safety filter를 결합했다.

### 단계 B. bbox는 보이지만 robot target은 틀리는 문제

관찰:

- object detection 결과가 좋아도 로봇이 접근할 좌표는 depth 및 좌표 변환 오차에 민감하다.
- 둥근 작은 물체와 배경 wall이 함께 bbox에 들어오면 단일 depth sample은 불안정하다.

구현된 대응:

- red pixel 우선 depth surface estimation
- near depth percentile 기반 배경 억제
- eye-in-hand transform과 현재 joint FK를 이용한 `base_link` 변환
- EMA target smoothing과 target lock

배운 점:

> 인식 정확도와 grasp target 정확도는 별도의 지표다. 실제 manipulation에서는 bbox mAP보다 target position error, 접근 성공률, 손상 없는 파지율이 더 직접적인 KPI가 된다.

### 단계 C. 목표 좌표는 있어도 팔이 안정적으로 못 가는 문제

관찰:

- 목표 Cartesian pose가 동일해도 IK solution branch와 wrist posture에 따라 실제 움직임의 안전성이 달라진다.
- Doosan native 이동만으로 빠르게 실행할 수 있지만 collision-aware global route와 branch 통제가 필요했다.

구현된 대응:

- cuRobo `MotionGen`으로 approach/grasp/retreat 분리 계획
- planning 성공 후 operational joint range 검사
- joint target 이동에는 joint-space planning 경로 추가
- 별도 `pick_place_node.py`로 native MoveJointX/MoveLine baseline도 남김

배운 점:

> 플래너를 붙이는 일은 API 연결이 아니라, robot model, tool frame, joint limit, 실행기 시간 파라미터, 실패 복구 조건을 같은 기준으로 맞추는 시스템 통합 작업이다.

### 단계 D. 실제 환경 충돌과 데모 안정화

관찰:

- 실제 wall 위치와 planner world가 어긋나면 소프트웨어상 안전한 경로도 실물에서는 위험하다.
- 반대로 coarse collision model을 과도하게 넣으면 정상 자세도 충돌로 판정되어 계획 자체가 막힌다.

구현된 대응:

- 실측 whiteboard surface를 바탕으로 wall cuboid 생성
- `INVALID_START_STATE_WORLD_COLLISION` 시 단일 obstacle 진단 함수 추가
- attached strawberry spheres 구조 추가
- place 과정의 좌/우 위험 구역에 heuristic transfer 적용

솔직히 밝혀야 할 한계:

- self collision check가 꺼져 있다.
- tray와 table collision이 꺼져 있다.
- 이미 놓은 딸기 obstacle도 꺼져 있다.
- 즉 현재 world는 생산용 안전 모델이 아니라 시연을 진행하기 위한 단계적 모델이다.

### 단계 E. Place 및 그리퍼 다루기

구현된 대응:

- 계란판의 각 목적지를 `above`와 `release`로 분리했다.
- TCP와 joint를 기록하는 teaching tool을 만들었다.
- slot0, slot1, slot2에는 실측 pose가 저장되어 있다.
- 그리퍼는 300 -> 420 -> 580 position의 soft close를 사용한다.

한계:

- 계란판이 움직이면 재티칭해야 한다.
- stroke는 접촉 센서가 아니므로 딸기 유무/손상 여부를 판단하지 않는다.
- force/current 또는 place 전후 vision 검증이 다음 단계의 핵심이다.

## 5. 현재 코드의 강점과 기술 부채

### 5.1 강점

1. Perception에서 execution까지 ROS 인터페이스가 실제로 이어져 있다.
2. 실패가 발생했던 지점을 config/로그/진단 코드로 남겨 반복 실험이 가능해졌다.
3. 환경 YAML을 visualizer와 cuRobo가 함께 읽어 world model 정합을 시작했다.
4. 하드웨어가 없는 경로도 URDF/Gazebo/virtual mode와 native baseline으로 일부 확보되어 있다.
5. 팀 vision branch에는 확대된 detector 데이터와 줄기 segmentation 실험 산출물이 있어 후속 프로젝트 자산이 된다.

### 5.2 포트폴리오에 숨기지 말아야 할 부족한 점

| 우선순위 | 문제 | 왜 중요한가 | 개선 |
| --- | --- | --- | --- |
| P0 | `pick_complete`가 실제 수확 성공을 뜻하지 않는다 | planner abort 후에도 일부 경로에서 publish되고, vision 로그는 이를 `sequence_complete`로 기록한다 | result message/action으로 `SUCCESS`, `PLAN_FAIL`, `GRASP_UNVERIFIED`, `PLACE_FAIL` 분리 |
| P0 | collision safety가 부분 비활성화 상태다 | 농장형/제품형 안전성을 주장할 수 없다 | self-collision sphere 재모델링, tray/table/fruit enable, signed-distance/scene 동기화 검증 |
| P0 | 파지 성공/압상 판정 센서가 없다 | 실제 딸기는 손상 여부가 핵심 품질이다 | motor current/force sensing 또는 soft tactile, RGB-D post-check, damage score 기록 |
| P1 | 두 대형 Python 노드에 정책과 알고리즘이 밀집되어 있다 | 실험 변경이 regression을 만들기 쉽다 | detector/depth/transform/task/planner/executor 분리 |
| P1 | 전역 상수와 hard-coded home/wall 값이 많다 | 환경 변경 시 재현과 튜닝 이력 관리가 어렵다 | ROS parameter YAML + experiment profile + calibration/run manifest |
| P1 | auto mode가 성공과 실패를 구별하지 못한다 | 실패 target을 완료로 block하거나 일부 abort에서 대기 상태에 멈출 수 있다 | 명시적 state machine과 retry/skip 정책 |
| P1 | 계란판 pose가 고정 티칭값에 의존한다 | tray가 이동/회전하면 place target과 collision scene이 함께 틀어진다 | AprilTag/ArUco tray frame 추정 후 RGB-D hole/grid 보정, slot 자동 생성 |
| P1 | launch/README가 과거 DINO 설명과 현재 YOLO 실행을 혼용한다 | 발표 재현성과 협업성이 떨어진다 | 주 실행 경로 README 재작성, legacy 실험을 별도 문서로 격리 |
| P2 | `regions.yaml`은 실행 코드와 연결되지 않았다 | quadtree 계획을 구현 성과처럼 말할 위험 | scan manager node와 persistent region state 구현 |

특히 성공률을 계산하려면 현재 JSONL 구조를 먼저 고쳐야 한다. `pick_complete`는 “sequence가 끝나거나 종료 신호가 발생함” 정도이지 “딸기 수확 성공”이 아니다.

## 6. 코드를 그대로 이어갈 것인가

결론: **현재 코드를 버리지 말고, 데모 기준선으로 동결한 뒤 실전 프로젝트용 구조를 새 패키지/새 branch에서 점진적으로 분리하는 것이 맞다.**

이유:

1. 현재 코드는 실물 로봇, 좌표계, service 이름, 티칭 pose, 시행착오 파라미터가 축적된 귀중한 baseline이다.
2. 전면 재작성은 기존에 해결한 calibration/execution 문제를 다시 겪을 가능성이 높다.
3. 반대로 그대로 기능만 추가하면 두 대형 노드와 실험 flag가 감당할 수 없게 된다.

권장 git/개발 전략:

```text
demo/2026-05-strawberry-board     현재 성공 시연 재현 가능한 snapshot 태그
feature/runtime-result-contract   성공/실패 event와 로그 스키마 정리
feature/modular-perception        detection/depth/transform 분리
feature/modular-task-planning     state machine/planner/executor 분리
feature/realistic-fruit-cell      실제형 모형, 동적 scene, 측정 실험
feature/quadtree-scan             영역 탐색
feature/vla-supervisor            semantic 판단 실험
```

새 구조 제안:

```text
harvest_interfaces/
  msg/TargetCandidate.msg
  msg/HarvestResult.msg
  action/HarvestTarget.action

perception/
  strawberry_detector_node.py
  depth_surface_estimator.py
  target_tracker.py
  maturity_filter.py

calibration/
  transform_provider.py
  calibration_validator.py

scene/
  scene_manager_node.py
  tray_localizer.py
  slot_occupancy_estimator.py
  collision_world_bridge.py

planning/
  planner_adapter.py
  curobo_adapter.py
  moveit_adapter.py
  native_motion_baseline.py

task/
  harvest_state_machine_node.py
  retry_policy.py
  slot_manager.py
  scan_manager.py

intelligence/
  vla_supervisor_node.py

evaluation/
  run_logger.py
  benchmark_runner.py
  metrics_report.py
```

처음부터 이 전체를 다 만들지 말고, 첫 리팩터링은 `HarvestResult` 계약과 logger부터 시작한다. 성공/실패 측정이 없으면 이후 플래너 개선도 비교할 방법이 없다.

## 7. MoveIt/cuRobo와 현업 사용에 대한 정확한 답변

### 7.1 들은 이야기의 맞는 부분

현업 셀에서는 “설치한 기본 planner를 아무 수정 없이 버튼 하나로 생산 투입”하는 경우는 드물다. 실제 시스템은 다음을 제품/셀 조건에 맞게 커스텀한다.

- collision geometry 및 안전 margin
- fixture, tool, payload, attached object
- 작업 state machine, retry, recovery, fail-safe
- candidate selection 및 grasp strategy
- vendor controller에 trajectory를 넘기는 실행층
- cycle time, jerk, singularity, branch, human safety 조건
- 로그, 모니터링, operator intervention

### 7.2 틀리거나 과도한 일반화

“MoveIt이나 cuRobo는 현업에서 메인으로 거의 쓰지 않고 알고리즘 검증용일 뿐”이라고 말하면 위험하다.

- MoveIt 공식 문서는 MoveIt이 planner plugin 구조이며 OMPL, Pilz industrial planner, CHOMP 및 custom constraints/IK를 지원한다고 설명한다.
- MoveIt 공식 application 페이지는 상업 배포용 MoveIt Pro와 material handling, ML-powered bin picking 사례를 제시한다.
- cuRobo 공식 페이지는 commercial application을 위한 motion planner가 Isaac ROS cuMotion의 MoveIt plugin 형태로 제공된다고 명시한다.
- NVIDIA Isaac ROS 문서는 cuMotion planner node의 motion generation이 MoveIt 2 plugin으로 노출된다고 설명한다.
- 2025년 말 이후 NVIDIA의 `cuMotion` 문서는 cuRobo에서 도입된 collision-aware IK/trajectory optimization 알고리즘의 hardened 구현과 end-to-end motion generation을 제공한다고 설명한다.

즉 면접 답변은 다음이 가장 정확하다.

> 현업에서는 MoveIt/cuMotion/cuRobo 같은 범용 프레임워크를 그대로 믿고 끝내지도 않고, 무조건 버리고 전부 재작성하지도 않습니다. 검증된 planning/scene/IK 기반 위에 해당 작업의 tool geometry, perception uncertainty, safety rule, retry policy, controller interface를 커스텀합니다. 제 프로젝트에서도 cuRobo는 경로 생성 엔진으로 활용하고, 딸기 작업에 필요한 wall 접근 자세, depth 오류 대응, slot 배치, 실패 처리와 충돌 모델을 직접 설계하는 방향으로 고도화하겠습니다.

## 8. 기존 플래너 한계 분석과 개선 연구 방법

직접 알고리즘을 새로 만든다고 시작하지 말고, **비교 가능한 실패 데이터셋을 만든 후 병목에만 개선을 적용**한다.

### 8.1 연구 질문

1. 실패는 perception target error 때문인가, IK/계획 때문인가, 실행/파지/충돌 모델 때문인가?
2. 현재 cuRobo 경로는 native MoveJointX/MoveLine 또는 MoveIt/Pilz/OMPL 대비 어디서 낫고 어디서 나쁜가?
3. 실제 과실/잎/줄기/tray를 반영하면 planning success와 안전 clearance가 어떻게 변하는가?
4. 빨리 계획하는 것과 과실을 손상 없이 안전하게 수확하는 것 사이에 어떤 trade-off가 있는가?

### 8.2 반드시 먼저 수집할 지표

| 범주 | 지표 |
| --- | --- |
| 인식 | class precision/recall, maturity error, depth-valid rate, target 3D error(mm) |
| 계획 | plan success rate, planning latency(ms), IK failure reason, path length, joint travel, minimum clearance |
| 실행 | controller reject/timeout, execution time(s), actual trajectory deviation |
| 수확 | grasp success, detach success, place success, drop rate, fruit damage score |
| 시스템 | end-to-end cycle time, recovery count, human intervention rate |

3D target error는 마커가 붙은 reference fruit 또는 측정 가능한 fixture를 사용해 ground truth를 만들고, perception 개선과 planner 개선을 혼동하지 않도록 별도 측정한다.

### 8.3 Benchmark scene 구성

| Scene ID | 조건 | 확인하려는 한계 |
| --- | --- | --- |
| S0 | 단일 정면 딸기, 장애물 없음 | 좌표계 및 baseline |
| S1 | 좌/중/우, 상/하 workspace grid | IK branch, reachability |
| S2 | wall에 가까운 여러 grasp depth | collision margin, grasp offset |
| S3 | 주변 딸기/잎 모형 장애물 | approach/retreat clearance |
| S4 | 줄기/잎으로 부분 가림 | perception/graspability |
| S5 | 이동한 tray/occupied slots | scene update/place planning |
| S6 | 조명/반사/과실 크기 변화 | robustness |

각 scene에서 동일 target을 다음 planner 정책으로 실행하거나 offline planning한다.

| Baseline | 의미 |
| --- | --- |
| B0 Native | `pick_place_node.py` 기반 Doosan MoveJointX/MoveLine |
| B1 Current cuRobo | 현재 데모 설정 |
| B2 Strict cuRobo | self/table/tray/attached/placed object를 켠 정교화 설정 |
| B3 MoveIt baseline | Planning Scene + OMPL 또는 Pilz 산업용 동작 |
| B4 Hybrid/custom | 선정된 planner + 직접 만든 task constraint/retry/scene policy |

`S5`에서는 fixed teaching pose, marker-derived tray pose, marker+RGB-D-refined tray pose로 생성한 place target을 함께 비교한다.

### 8.4 실패 taxonomy

로그 이벤트를 다음으로 고정한다.

```text
PERCEPTION_NO_TARGET
PERCEPTION_DEPTH_INVALID
TARGET_OUT_OF_WORKSPACE
IK_FAIL
START_STATE_COLLISION
PATH_COLLISION_OR_MARGIN_REJECT
BRANCH_OR_SINGULARITY_REJECT
EXECUTION_REJECTED
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

### 8.5 개선 아이디어는 병목별로 선택

| 발견된 병목 | 우선 개선 |
| --- | --- |
| target 오차가 지배적 | segmentation surface point, multi-view depth fusion, calibration validation |
| IK branch가 지배적 | preferred joint region cost, seed policy, approach orientation sampling |
| wall/leaf 충돌이 지배적 | dense scene model, signed-distance clearance cost, constrained retreat |
| path는 되나 딸기가 손상 | gripper/수확 방식 개선, force/current closed loop, 줄기 cutting/grasp 전략 |
| cycle time이 병목 | warm start, parallel candidate planning, scan sequence 최적화 |

좋은 연구 제목:

> RGB-D 수확 작업에서 인식 불확실성과 작업별 안전 제약을 반영한 모션 플래닝 정책 비교 및 개선

이 제목은 “cuRobo를 내가 대체한다”보다 현실적이며, 분석/개선의 깊이를 보여준다.

## 9. 실제에 가까운 딸기 모형 프로젝트 실행 로드맵

### Phase 0. 현재 데모 동결과 측정 기반 마련 (1주)

완료 기준:

- 재현 가능한 git tag/branch 생성
- `HarvestResult`와 failure taxonomy 저장
- 영상, config snapshot, calibration ID, 코드 commit hash가 한 run 폴더에 남음
- 현재 보드 데모에서 최소 30회 결과 라벨 확보

이 단계 없이 새 모형으로 가면 문제가 생겼을 때 이전보다 나빠졌는지 판단할 수 없다.

### Phase 1. 실제형 모형과 scene 정교화 (1~2주)

모형은 빨간 덩어리 하나가 아니라 다음 요소를 포함해야 한다.

- 과실 크기/익음 정도/표면 반사 차이
- 꼭지와 줄기
- 유연한 잎 가림
- 서로 가까운 과실 cluster
- 흔들림 또는 약한 compliance
- 이동 가능한 수확 tray
- marker 부착, marker 부분 가림, marker 미검출 조건을 포함한 tray localization 평가 환경

개발 항목:

- fruit/stem/leaf/tray semantic object 표현
- table/tray/self-collision 다시 활성화할 수 있는 robot sphere 검증
- tray pose를 AprilTag/ArUco 또는 corner feature로 추정
- 추정한 `tray_frame` 기준으로 slot center, `above`, `release` pose를 자동 생성
- RGB-D hole pattern/point cloud grid로 marker pose refinement 및 slot occupancy 검증
- fruit damage 평가 기준: 눌림, 낙하, 미수확, 꼭지 손상

### Phase 2. 모듈화와 상태 기계 (1~2주)

작업 상태:

```text
IDLE -> SCAN -> SELECT -> VALIDATE_TARGET -> PLAN_APPROACH
 -> APPROACH -> GRASP -> VERIFY_GRASP -> RETREAT
 -> PLAN_PLACE -> PLACE -> VERIFY_PLACE -> UPDATE_MAP -> SCAN
 -> RECOVER or DONE
```

이 단계의 핵심은 planner 종류를 바꾸더라도 task logic과 metrics가 그대로 유지되도록 하는 것이다.

### Phase 2.5. 계란판 비전/마커 인식과 자동 Place (1~2주)

현재 계란판은 `place_slots.yaml`의 고정 티칭 pose에 의존한다. 실전 프로젝트에서는 tray 이동과 회전을 허용하기 위해 다음 순서로 개선한다.

1. 계란판의 강체 부위에 AprilTag 또는 ArUco marker를 부착하고 RealSense에서 pose를 추정한다.
2. eye-in-hand 변환으로 `base_link -> tray_frame`을 계산한다.
3. tray 규격의 row, column, pitch를 이용해 slot center와 `above`/`release` 목표를 자동 생성한다.
4. tray body 및 occupied slot/placed fruit를 MoveIt scene과 cuRobo world에 갱신한다.
5. RGB-D의 hole pattern, 외곽 또는 point cloud grid로 marker pose를 보정하고, place 후 occupancy를 확인한다.
6. marker-only, marker+vision refinement, vision-only 조건을 비교한다.

| 지표 | 의미 |
| --- | --- |
| tray pose translation/rotation error | 이동한 계란판 frame 복원 정확도 |
| slot center error | 생성한 place 목표와 실제 hole 중심의 차이 |
| relocalization success | tray 이동 후 재배치 성공률 |
| occupancy classification accuracy | 채워진 slot을 회피하는 정확도 |
| place/drop success | 자동 생성 목표의 실기 유효성 |

처음부터 markerless vision-only를 고집하지 않는다. marker baseline으로 좌표계와 planner 연동을 검증한 뒤, 오염/가림 상황에 대비한 RGB-D geometry 보정을 추가하는 것이 합리적이다.

### Phase 3. Quadtree 기반 탐색 (1~2주)

quadtree는 trajectory planner가 아니라 **관측/작업 영역 스케줄러**로 둔다.

cell state:

```text
UNSEEN, EMPTY, RIPE_CANDIDATE, UNRIPE, OCCLUDED, FAILED, DONE
```

동작:

1. 먼저 coarse 2x2 scan pose를 티칭/자동 생성한다.
2. ripe 후보가 밀집되거나 불확실한 cell만 분할한다.
3. target이 사라지거나 실패한 cell은 재관측 비용을 높이고 일정 시간 후 재시도한다.
4. done/empty cell 중복 관측을 줄여 전체 cycle time을 측정한다.

비교 지표:

- 전체 대상 발견 recall
- scan pose 수
- 중복 관측 비율
- 전체 수확 cycle time
- 실패 후 재발견율

### Phase 4. MoveIt과 cuRobo/cुMotion 역할 분담 (2주)

추천 구조:

| 역할 | 도구 |
| --- | --- |
| URDF/SRDF, planning scene, TF, 환경 시각화, baseline planner | MoveIt 2 |
| GPU 기반 빠른 collision-aware motion generation 비교 | 현재 cuRobo 또는 Isaac ROS cuMotion 검토 |
| 딸기 task state, target uncertainty, retry/safety policy | 직접 작성한 task layer |
| 실제 실행 | Doosan controller interface adapter |

중요한 실험:

- 동일 scene/goal에서 MoveIt OMPL/Pilz와 cuRobo policy를 offline 비교
- MoveIt Planning Scene과 cuRobo world에서 동일 obstacle의 pose/clearance 일치 테스트
- tray localizer가 갱신한 tray body, occupied slot, placed fruit obstacle을 양쪽 world에 동일하게 반영하는 테스트
- 가능하다면 Isaac ROS cuMotion의 MoveIt 2 plugin 방식과 현재 직접 서비스 연결 방식을 비교

### Phase 5. VLA는 supervisor부터 적용 (2~4주)

VLA를 즉시 joint command 생성기로 사용하면 안전성 검증이 어렵다. 첫 적용은 고수준 판단에 제한한다.

초기 VLA 입력/출력:

```text
입력: RGB crop + detector/segmenter 결과 + cell 상태 + 질문
출력: PICKABLE / OCCLUDED / UNRIPE / DAMAGED / REOBSERVE
      및 근거 텍스트와 confidence
```

VLA가 맡을 수 있는 일:

- 색상 rule만으로 모호한 숙도/손상/가림 분류
- 잎이 접근 경로를 막고 있는지 semantic veto
- 실패 이미지의 원인 태깅 보조
- quadtree에서 재관측 우선순위 제안

VLA가 처음부터 맡지 말아야 할 일:

- safety-certified collision checking 대체
- raw trajectory 직접 실행
- force/압상 확인 없는 grasp 성공 선언

검증 방법:

- 사람이 라벨한 `pickable/occluded/unripe/damaged` test set을 먼저 만든다.
- rule-only, detector+rule, detector+VLA supervisor를 동일 데이터에서 비교한다.
- VLA 판단이 틀려도 motion safety layer가 실행을 거부할 수 있어야 한다.

## 10. rqt graph 및 런타임 점검 기준

현재 주 파이프라인에서 확인해야 할 연결:

```text
/dsr01/joint_states
  -> strawberry_yolo_node
  -> curobo_planner_node

strawberry_yolo_node
  -> /dsr01/curobo/pick_pose
  -> curobo_planner_node

curobo_planner_node
  -> /dsr01/motion/move_spline_joint
  -> /dsr01/motion/move_joint
  -> /dsr01/gripper/open
  -> /dsr01/gripper/position_cmd
  -> /dsr01/curobo/pick_complete

gripper_service_node
  -> /dsr01/gripper/stroke
  -> curobo_planner_node
```

실험 전 체크리스트:

1. `joint_states`에 arm joint 6개 이름과 최신 timestamp가 들어오는가.
2. `pick_pose.header.frame_id`가 `base_link`로 일관되는가.
3. calibration file ID와 현재 카메라 장착 상태가 맞는가.
4. 환경 visualizer의 wall/tray marker와 실제 치수가 맞는가.
5. planner가 사용한 collision enable 목록이 run log에 저장되는가.
6. `pick_complete` 대신 상세 result code를 수집하는가.
7. rqt graph 캡처와 rosbag/attempt image를 같은 실험 ID로 보관하는가.

## 11. 포트폴리오에 넣을 구성

### 11.1 첫 페이지 요약

**프로젝트:** RGB-D 비전과 GPU 모션플래닝 기반 딸기 수확 로봇 프로토타입
**역할:** 인식 좌표-로봇 실행 연결, 모션 시퀀스/충돌 환경/그리퍼/place 티칭 통합 및 실험 분석
**기술:** ROS 2 Humble, Doosan E0509, RH-P12-RN-A, RealSense, YOLO, eye-in-hand calibration, cuRobo, MoveIt/RViz, Python/C++
**핵심 성과:** 딸기 후보 검출부터 실제 로봇 pick/place 시퀀스 실행까지의 end-to-end pipeline 구현 및 실기 실험 로그 축적
**증거:** 3일간 168 target attempts, 111 sequence completion events, 수동 라벨 성공 2/실패 3; 별도 영상이 있는 경우 연속 배치 시연 첨부
**후속 확장:** marker/RGB-D 기반 계란판 pose 추정과 slot 자동 생성으로 고정 티칭 의존성 제거

### 11.2 핵심 문제 해결 카드

| 문제 | 분석 | 조치 | 배운 점 |
| --- | --- | --- | --- |
| bbox는 맞지만 접근점이 빗나감 | 중심 depth가 wall/노이즈를 읽음 | red-mask surface depth와 eye-in-hand FK 변환 | detection metric과 manipulation target metric은 다름 |
| 같은 목표에서 자세가 불안정 | IK branch/joint range/벽면 자세 영향 | cuRobo waypoint 계획 및 operational joint filter | 실제 팔은 pose만이 아니라 configuration 관리가 필요 |
| 이송 중 wall/tray 위험 | 실물과 collision model 불일치 | 실측 wall world, collision diagnostics, safe transfer | world model의 정합이 planner 이름보다 중요 |
| 딸기 파지가 거침 | 단일 close는 압상/실패 위험 | 단계적 position soft close | 농산물은 성공/실패 외 품질 지표가 필요 |

### 11.3 성과 문구: 증거 수준에 따른 선택

영상까지 제출할 경우:

> RealSense-YOLO 기반 딸기 인식, hand-eye 좌표 변환, cuRobo 경로 계획과 Doosan 실행 서비스를 통합하여 모형 딸기 3개를 계란판 slot에 순차 배치하는 실기 시연을 완료했습니다. 과정에서 깊이 오차, IK branch, wall collision, soft grasp 문제를 단계별로 분리하고 개선했습니다.

코드/로그만 제출할 경우:

> RealSense-YOLO 기반 딸기 target 생성부터 cuRobo/Doosan 기반 pick-place 실행까지 end-to-end 프로토타입을 구현했습니다. 3일간 168회의 target attempt와 111회의 sequence completion 이벤트를 기록했으며, 다음 단계에서는 결과 라벨링과 충돌/파지 검증을 강화해 정량 성공률을 확보하고 있습니다.

## 12. 자기소개서용 재료

### 12.1 문제 해결 경험 문단

저는 Doosan E0509 로봇팔과 RealSense RGB-D 카메라, YOLO, cuRobo를 연결해 딸기 수확 pick-and-place 프로토타입을 구현했습니다. 처음에는 화면에서 딸기를 검출하면 작업이 곧 완료될 것이라 생각했지만, bbox 내부 depth가 배경 보드를 읽어 실제 접근점이 어긋났고, 좌표를 보정한 뒤에도 IK branch와 collision world 불일치로 안전한 움직임을 만들기 어려웠습니다. 저는 문제를 인식, 좌표 변환, 경로 계획, 실행, 파지 결과의 단계로 나누었습니다. 빨간 표면 중심의 depth 추정과 hand-eye/FK 변환을 적용하고, cuRobo로 approach-grasp-retreat를 나누어 계획했으며, 실측 wall 모델과 계란판 place 티칭, 단계적 gripper close를 연결했습니다. 이 경험을 통해 실제 로봇 시스템에서는 하나의 알고리즘 성능보다 오차가 전달되는 전체 흐름을 측정하고 실패 원인을 분리하는 역량이 중요하다는 것을 배웠습니다.

### 12.2 성장/후속 목표 문단

현재 프로토타입은 흰 보드와 모형 딸기 환경에서 작업 흐름을 검증한 단계이며, collision object 일부와 파지 성공 센서가 아직 충분하지 않습니다. 저는 이를 숨기기보다 다음 연구 문제로 정의했습니다. 실제형 과실과 잎/줄기/가림 환경을 구성하고, MoveIt planning scene과 GPU 기반 motion generation을 비교 가능한 benchmark로 평가하며, quadtree로 탐색 이력을 관리하고 VLA는 수확 가능성 판단의 supervisor로 제한해 안전한 확장을 시도하고자 합니다. 목표는 단순 시연이 아니라, 왜 실패했는지 설명하고 개선 효과를 수치로 제시할 수 있는 농업 로봇 시스템을 만드는 것입니다.

### 12.3 Claude에 넘길 자기소개서 작성 프롬프트

```text
아래 사실만 근거로 로봇/스마트팜/자동화 직무 자기소개서의 문제해결 경험 문항을 작성해 줘.
과장하지 말고, 구현 사실과 향후 계획을 구분해 줘.

- 프로젝트: Doosan E0509 + RH-P12-RN-A + RealSense eye-in-hand + YOLO + cuRobo 딸기 수확 프로토타입
- 구현: YOLO ripe/unripe 검출, HSV ripe safety filter, bbox red-surface depth 추정, hand-eye+FK base 좌표 변환, cuRobo approach/grasp/retreat 계획, Doosan MoveSplineJoint 실행, gripper soft close, 계란판 slot above/release 티칭, wall collision world, 실험 이미지/JSONL logging
- 해결한 문제: bbox 중심 depth가 배경을 읽는 문제, IK branch/joint range 문제, wall model 불일치, 계란판 place 자세, 부드러운 파지
- 수치로 확인된 로그: 2026-05-18/19/21 target attempt 168건, sequence completion event 111건, 수동 라벨 성공 2건/실패 3건
- 별도 시연 영상을 제출하는 경우에만 '정상 딸기 3개 순차 배치 시연'을 사용
- 한계: stroke가 실제 파지 센서가 아님, self/tray/table/placed fruit collision 일부 비활성, VLA/quadtree는 향후 계획
- 후속 목표: 실제형 모형, 정량 benchmark, modular state machine, MoveIt scene+cuRobo/cुMotion 비교, quadtree scan, VLA supervisor
- 계란판 확장 목표: AprilTag/ArUco 기반 tray frame 추정, RGB-D pose refinement, slot/occupancy 자동 관리

문체: 실패 원인을 레이어별로 분해하고 실물 검증으로 개선하는 엔지니어임이 드러나게. 700~900자.
```

## 13. 면접 예상 질문과 답변 요지

### Q1. 이 프로젝트의 본질적인 난점은 무엇이었습니까?

객체 검출이 아니라 검출 결과를 물리적으로 잡을 수 있는 target과 안전한 실행으로 바꾸는 일이었습니다. depth 표면 추정, eye-in-hand 좌표계, IK branch, collision model, gripper 손상 문제가 직렬로 연결되어 있어 레이어별로 실패를 분리했습니다.

### Q2. 왜 YOLO bbox 중심 depth를 그대로 쓰지 않았습니까?

둥근 딸기와 뒤 wall이 하나의 bbox에 섞여 중심 픽셀이 배경이나 hole을 읽을 수 있었습니다. 그래서 bbox의 빨간 표면 픽셀을 우선하고 near-depth percentile로 앞 표면을 추정했습니다.

### Q3. 좌표 변환은 어떻게 합니까?

aligned depth와 camera intrinsic으로 camera-frame 3D point를 만든 뒤, eye-in-hand calibration의 `T_cam_to_gripper`와 현재 joint FK로 얻은 `T_gripper_to_base`를 곱해 base-frame target으로 변환합니다.

### Q4. 왜 cuRobo를 선택했습니까?

실기에서 Cartesian 목표에 대한 collision-aware trajectory와 다양한 IK seed를 빠르게 비교하기 위해서였습니다. 다만 cuRobo 자체가 모든 문제를 해결하는 것이 아니라 wall pose, tool geometry, branch 제한, 실행 service와의 연결을 직접 맞춰야 했습니다.

### Q5. MoveIt을 쓰지 않았다는 의미입니까?

주 실행 planner는 현재 cuRobo이지만 MoveIt/RViz 메시지 기반 환경 시각화를 추가했습니다. 후속 프로젝트에서는 MoveIt Planning Scene을 공통 환경 모델과 baseline planner로 사용하고, cuRobo 또는 cuMotion을 빠른 planning 후보로 공정하게 비교할 계획입니다.

### Q6. 현업은 MoveIt/cuRobo를 안 쓴다고 생각합니까?

그렇게 일반화하지 않습니다. 공식 자료에서도 MoveIt은 상업 적용과 custom plugin을 지원하고, NVIDIA의 cuMotion은 MoveIt 2 plugin으로 제공됩니다. 현업에서 직접 만드는 부분은 작업별 scene, safety/recovery, grasp 정책, controller integration, 필요 시 custom constraint/planner입니다.

### Q7. collision avoidance가 완성됐습니까?

아닙니다. wall collision과 진단 기반은 구현했으나, 현 데모에서는 coarse 모델의 false collision을 줄이기 위해 table/tray/self/placed fruit의 일부가 비활성화되어 있습니다. 생산 수준으로 가려면 이 부분을 다시 모델링하고 clearance를 정량 검증해야 합니다.

### Q8. 파지가 성공했다는 것을 어떻게 압니까?

현재 자동 판정은 충분하지 않습니다. `stroke`는 명령값에 가깝고 실제 접촉/힘 피드백이 아니어서 비활성화했습니다. 현재는 사람의 라벨/영상으로 확인하며, 후속에서는 current/force 또는 비전 기반 post-grasp 검증을 추가할 계획입니다.

### Q9. 로그상 성공률은 얼마입니까?

현재 로그에는 target attempt 168건, sequence completion event 111건이 있으나 completion은 성공과 동일하지 않습니다. 수동 success 라벨은 2건뿐이므로 의미 있는 성공률은 아직 보고하지 않고, 다음 실험부터 result code와 라벨을 필수화하겠습니다.

### Q10. 실제 딸기로 바뀌면 무엇이 가장 먼저 깨집니까?

숙도/반사/가림에 따른 인식, 과실 표면의 깊이 추정, 파지 압상, 줄기 분리 성공 여부, 잎과 주변 과실 collision이 먼저 문제될 가능성이 큽니다. 그래서 planner 변경 전에 실제형 모형과 측정 KPI를 먼저 준비하겠습니다.

### Q11. Quadtree는 motion planning에 어떻게 연결합니까?

Quadtree는 로봇 관측 영역의 상태 관리와 scan scheduling에 사용하고, 실제 arm trajectory는 MoveIt/cuRobo 계층이 담당하게 분리합니다. 이로써 중복 관측과 실패 재시도를 관리하면서 planner와 탐색 로직을 독립적으로 비교할 수 있습니다.

### Q12. VLA를 왜 직접 제어기로 쓰지 않습니까?

실제 수확에서는 충돌과 손상에 대한 설명 가능하고 반복 가능한 safety gate가 우선입니다. VLA는 처음에는 가림/숙도/재관측 판단을 보조하는 supervisor로 사용하고, 기하학적 collision 및 실행 검증은 결정론적 계층에서 유지하겠습니다.

### Q13. 직접 플래너를 개선한다면 무엇부터 할 것입니까?

플래너 이름보다 실패 taxonomy와 benchmark scene을 먼저 만듭니다. 현재 cuRobo, native motion, MoveIt baseline을 동일 target/world에서 비교해 IK failure, clearance, planning time, 수확 품질 중 실제 병목을 찾고 그 지점에 orientation sampling, cost, scene update 또는 retry policy를 적용하겠습니다.

### Q14. 본인의 기여를 한 문장으로 말해 보세요.

인식 결과를 실제 팔이 실행할 수 있는 수확 시퀀스로 연결하고, 깊이 오차와 모션/충돌/파지 실패를 실물 로그 기반으로 분해해 개선하는 통합 엔지니어링을 수행했습니다.

### Q15. 계란판 위치가 바뀌면 어떻게 배치할 계획입니까?

현재 데모는 티칭된 slot pose를 사용하므로 계란판 이동에 취약합니다. 후속 프로젝트에서는 먼저 AprilTag/ArUco로 `tray_frame`을 추정하고 tray 규격으로 slot 목표를 자동 생성하겠습니다. 이후 RGB-D의 hole pattern 또는 point cloud geometry로 marker pose를 보정하고 occupancy를 확인하여, marker가 일부 가려지는 조건까지 평가하겠습니다.

## 14. 필요한 이론 학습 목록

| 영역 | 반드시 설명할 수 있어야 하는 질문 |
| --- | --- |
| 좌표계/FK | homogeneous transform 곱셈 순서와 eye-in-hand calibration 오차가 target에 어떻게 전파되는가 |
| Depth | RGB-depth alignment, deprojection, surface depth 통계량 선택 이유 |
| IK/Planning | IK branch, singularity, joint-space vs Cartesian motion, sampling vs optimization planner |
| Collision | sphere/cuboid/SDF 표현의 장단점, attached object, safety margin |
| Trajectory | path와 time-parameterized trajectory 차이, velocity/acceleration/jerk |
| ROS 2 | topic/service/action 선택, callback concurrency, TF/planning scene, rqt graph 해석 |
| Manipulation | graspability, compliance, force/current feedback, 농산물 압상 평가 |
| Evaluation | failure taxonomy, ablation, benchmark fairness, 정량 KPI |
| VLA | action model의 역할과 불확실성, safety supervisor와 low-level controller의 분리 |

## 15. 이 프로젝트에서 얻은 가장 좋은 회고

화면에서 딸기를 찾는 것은 시스템의 시작일 뿐이었다. 실제 로봇은 잘못 추정된 depth를 그대로 믿고 움직이고, 정확한 target을 받아도 다른 IK branch를 선택하거나 실제 wall과 다른 collision model 때문에 위험해질 수 있다. 또한 딸기를 들어 올렸다는 사실만으로는 손상 없이 수확했다는 의미가 되지 않는다. 이 프로젝트의 가장 큰 학습은 한 모듈의 정확도를 자랑하는 것이 아니라, 인식부터 파지 품질까지 연결된 실패를 관측 가능하게 만들고 하나씩 분리해 고치는 태도였다.

## 16. 근거 파일

현재 workspace의 주요 근거:

- `docs/system_architecture.md`: 현재 architecture/sequence diagram
- `scripts/strawberry_yolo_node.py`: detection, depth, transform, tracking, attempt logging
- `scripts/curobo_planner_node.py`: cuRobo planning, execution, collision diagnostics, place sequence
- `scripts/environment_visualizer.py`, `config/environment.yaml`: wall/tray visual/collision world
- `scripts/teach_place_slots.py`, `config/place_slots.yaml`: 계란판 티칭 pose
- `scripts/pick_place_node.py`: cuRobo 없는 native baseline
- `src/gripper_service_node.cpp`: 실제 gripper 통신 및 stroke 발행 방식
- `logs/pick_attempts/2026-05-18`, `2026-05-19`, `2026-05-21`: 실험 기록
- git remote branch `team/feature/vision_yolo26m_strawberry`: detector/stem 학습 자료와 지표
- git remote branch `team/feature/calibration`: calibration 산출물 계보
- git branch `feature/motion`: motion pipeline 문서/기초 구현 계보

외부 확인 자료:

- MoveIt Concepts, planner plugins/planning scene/adapters: <https://moveit.ai/documentation/concepts/>
- MoveIt Applications, commercial/production-oriented examples: <https://moveit.ai/documentation/applications/>
- NVIDIA cuRobo official site: <https://curobo.org/index.html>
- NVIDIA cuRobo ICRA publication summary: <https://research.nvidia.com/publication/2023-05_curobo-parallelized-collision-free-robot-motion-generation>
- Isaac ROS cuMotion, MoveIt 2 plugin interface: <https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion/index.html>
- NVIDIA cuMotion documentation: <https://nvidia-isaac.github.io/cumotion/index.html>
- OpenVLA project: <https://openvla.github.io/>
- OpenVLA source repository: <https://github.com/openvla/openvla>
