# 수확 런타임 파이프라인과 시뮬레이션 재생 로그

## 1. 전체 실행 구조

현재 주 실행 경로는 다음과 같다.

```text
RealSense RGB + aligned depth
 -> strawberry_fusion_node
    -> YOLO segmentation: ripe / unripe / sick
    -> YOLO pose: KP0 stem_base / KP1 stem_mid / KP2 stem_tip
    -> seg-pose matching + HSV ripe safety filter
    -> stem keypoint confidence/depth/geometry quality guard
    -> keypoint depth + eye-in-hand calibration + E0509 FK
    -> stable base_link stem target
 -> /strawberry/detection/pick_pose
 -> scan_executor_node
    -> 현재 root cell의 후보 버퍼링
    -> 중복 제거 및 logical nw/ne/se/sw 순서 결정
    -> 한 번에 하나의 target만 planner로 전달
 -> /dsr01/curobo/pick_pose
 -> curobo_planner_node
    -> collision world에 whiteboard/이웃 딸기 반영
    -> pre-approach 경로 생성
    -> grasp endpoint IK/collision/branch 검증
 -> Doosan MoveSplineJoint: scan pose -> pre-approach
 -> Doosan MoveLine TOOL +Z: 정지 후 최종 직선 진입
 -> RH-P12-RN-A close
 -> Doosan MoveLine BASE -Z: 40mm detach pull
 -> Doosan MoveLine TOOL -Z: 추가 진입 거리 직선 역주행
 -> VERIFY_GRASP: gripper present position 기반 접촉/빈 파지 판정
 -> [미구현] VERIFY_DETACH / RETAINED_AFTER_RETREAT
 -> optional guarded marker place
    -> overview -> tray-view -> marker slot above
    -> explicit release 승인 시 descend/open/above
 -> cuRobo joint-space plan + MoveSplineJoint: 현재 cell의 pick 시작 scan pose 복귀
 -> /dsr01/curobo/pick_complete
 -> scan_executor_node가 같은 cell의 다음 target 또는 다음 cell 진행
 -> 전체 순회 종료 시 scan_executor_node가 overview 복귀
```

## 2. 단계별 책임

| 단계 | 사용 기술 | 출력 / 판단 |
| --- | --- | --- |
| RGB-D 취득 | RealSense D435 | color image, aligned depth |
| 숙도/상태 판단 | YOLO segmentation + HSV | ripe만 수확 후보, unripe/sick 제외 |
| 줄기 위치 판단 | YOLO pose | KP0/KP1/KP2 |
| seg-pose 결합 | `strawberry_fusion_node.py` | 같은 과실의 mask와 keypoint 매칭 |
| 3D 변환 | depth + hand-eye + E0509 FK | `base_link` 기준 줄기 좌표 |
| 목표 안정화 | 최근 9개 KP0 중앙값 + 12mm spread 제한 | 흔들리는 target 발행 차단 |
| 줄기 품질 검증 | KP confidence + depth + 3D segment geometry | 가림/오검출 target 접근 차단 |
| scan dwell | 첫 stable target까지 adaptive wait, 최대 12초 | 빠른 target은 즉시 진행하고 늦은 안정화 target race 방지 |
| 셀/타깃 순서 | `scan_executor_node.py` | root cell 및 logical subcell 순차 실행 |
| 장애물 구성 | `curobo_planner_node.py` | whiteboard + 이웃 딸기 sphere |
| pre-approach | cuRobo Cartesian planning | 충돌 회피 joint trajectory |
| grasp endpoint | cuRobo validation only | IK/collision/branch 실행 가능성 검사 |
| pre-approach 실행 | Doosan `MoveSplineJoint` | cuRobo trajectory를 실제 로봇에 실행 |
| 최종 진입 | Doosan `MoveLine` | TOOL `+Z` 직선 접근 |
| 파지 | RH-P12-RN-A gripper service | close 명령, 실제 파지 성공은 별도 검증 필요 |
| 분리 동작 | Doosan `MoveLine` | BASE `-Z 40mm`로 아래 방향 detach pull |
| 초기 후퇴 | Doosan `MoveLine` | 추가 진입 거리를 TOOL `-Z`로 역주행 |
| 파지 검증 | gripper present position | 접촉/빈 파지 판정 구현, hardware read와 오인 검증 필요 |
| 분리/유지 검증 | 현재 미구현 | 사람 관찰 외 자동 성공 근거 없음 |
| marker place | fresh tray JSON + cuRobo + MoveLine | 기본 비활성, preview/release 명시 승인형 |
| scan pose 복귀 | cuRobo joint-space + `MoveSplineJoint` | 같은 셀의 다음 target을 위해 pick 시작 자세로 복귀 |

## 3. cuRobo를 사용하는 구간

cuRobo는 GPU 기반 IK, trajectory optimization, collision checking에 사용한다.

현재 사용하는 구간:

1. 현재 scan pose에서 pre-approach까지 Cartesian 목표 계획
2. 최종 grasp endpoint의 IK 및 collision 가능성 사전 검증
3. operational joint limit, J1 branch swing, J4/J6 spline jump 검사
4. 직선 후퇴 완료 후 이번 pick이 시작된 cell scan pose까지 경로 계획

cuRobo 결과는 joint trajectory이며, 실제 로봇 실행은 Doosan
`MoveSplineJoint` 서비스가 담당한다.

Grasp 후보 탐색에서 pre-approach는 orientation별로 한 번만 계획하고, 같은
orientation의 grasp offset endpoint를 순차 검증한다. offset마다 동일 pre-approach를
재계획하지 않는다.

## 4. cuRobo를 사용하지 않는 구간

최종 줄기 접근과 초기 retreat에는 cuRobo trajectory를 사용하지 않는다.

```text
pre-approach -> grasp: TOOL +Z MoveLine
grasp -> detach: BASE -Z 40mm MoveLine
detach -> retreat: TOOL -Z MoveLine
```

이유:

- cuRobo joint trajectory는 TCP가 완전한 직선을 따라간다고 보장하지 않는다.
- 줄기 근처에서는 손목/베이스 branch 변경보다 자세를 유지한 직선 이동이 안전하다.
- retreat를 새 Cartesian 목표로 다시 계획하면 J1 반대 branch가 선택될 수 있다.

그리퍼 close/open도 cuRobo가 아니라 RH-P12-RN-A 서비스 계층이 수행한다.

## 5. 현재 구조의 중요한 한계

- `PICK COMPLETE`는 시퀀스 종료이지 실제 파지 성공이 아니다.
- planner는 각 pick 후 overview가 아니라 해당 pick의 시작 scan pose로 복귀한다.
  같은 셀의 다음 target 전달과 셀 이동/최종 overview 복귀는 scan executor가 담당한다.
- 실제 파지 성공은 현재 영상/사람 관찰로 별도 판정해야 한다.
- self-collision은 coarse sphere 오검출 때문에 현재 비활성 상태다.
- marker place는 guarded optional 경로로 연결되었지만, tray localization이 300초보다
  오래되거나 release가 명시 승인되지 않으면 place를 차단하고 현재 자세에서 정지한다.
- 현재 `grasp OK` 로그는 grasp 목표 자세 도달을 뜻하며 실제 파지/분리 성공이 아니다.
- `VERIFY_GRASP`는 구현됐지만 hardware read 실패 시 `GRASP_UNVERIFIED`로
  fail-open한다. `VERIFY_DETACH / RETAINED_AFTER_RETREAT`는 미구현이므로
  미분리 상태에서도 fresh tray target이 있으면 place 단계로 진행할 수 있다.
- 잎은 현재 perception class 및 collision world에 포함되지 않는다. 따라서 과실 sphere와
  whiteboard를 피한 경로라도 실제 잎과 접촉할 수 있다.
- 목표 위치가 안정적이어도 pose 모델이 KP0를 일관되게 잘못 찍으면 옆 접근할 수 있다.
  현재 quality guard는 명백한 저신뢰/비정상 geometry를 거부하지만, 일관된 오검출을
  완전히 판별하지는 못한다.

### 파지 성공 측정 계약

파지 성공은 하나의 이벤트가 아니라 다음 단계별 결과로 기록해야 한다.

```text
GRASP_POSE_REACHED
 -> GRASP_CONTACT_DETECTED | GRASP_EMPTY | GRASP_UNVERIFIED
 -> DETACH_SUCCESS | DETACH_FAIL | DETACH_UNVERIFIED
 -> RETAINED_AFTER_RETREAT | DROP_DURING_RETREAT | RETENTION_UNVERIFIED
 -> SUCCESS only when grasp + detach + retention are all verified
```

권장 KPI:

| KPI | 계산식 |
| --- | --- |
| verification coverage | 유효 gripper 판독 / close 시도 |
| contact detection rate | `GRASP_CONTACT_DETECTED` / 유효 gripper 판독 |
| empty grasp rate | `GRASP_EMPTY` / 유효 gripper 판독 |
| detach success rate | verified detach / grasp 시도 |
| retention success rate | retreat 후 유지 / verified detach |
| end-to-end harvest success | verified grasp+detach+retention / 전체 시도 |
| verifier precision/recall | 자동 파지 판정과 사람 라벨 비교 |

현재 `_verify_grasp()`는 present position `< 665`이면 접촉, `>= 665`이면 empty로
판정한다. 하지만 hardware read가 실패하면 `GRASP_UNVERIFIED`이며, 잎 접촉도
과실 파지로 오인할 수 있다. 따라서 position 판정만으로 `SUCCESS`를 선언하지
않고, retreat 후 비전 확인 또는 force/current/tactile 근거를 추가해야 한다.

## 2026-06-09 SW 단일 과실 체크포인트

- SW 단일 과실의 줄기 파지 및 분리 성공 사례를 사용자가 육안 확인했다.
- 최신 완료 run `20260609T160052-da5edd5a`는 target 수신부터 scan pose 복귀까지
  약 `36.4초`가 소요됐다.
- 현재 설정은 grasp Z `+30mm`, extra advance `65mm`, BASE `-Z 40mm` detach
  pull을 사용한다.
- 자동 판정은 gripper hardware read 실패로 `GRASP_UNVERIFIED`이므로, 이 결과를
  정량 성공률로 집계하지 않는다.

## 6. 시뮬레이션 재생용 JSONL 로그

### 저장 위치

```text
logs/runtime/YYYY-MM-DD/
  strawberry_fusion_node_<run_id>.jsonl
  curobo_planner_node_<run_id>.jsonl
```

환경 변수로 다른 위치를 지정할 수 있다.

```bash
export STRAWBERRY_RUNTIME_LOG_ROOT=/path/to/runtime_logs
```

### 공통 스키마

각 줄은 독립적인 JSON 객체다.

```json
{
  "schema_version": "strawberry_runtime_event.v1",
  "timestamp": "2026-06-08T14:30:00.123+09:00",
  "monotonic_sec": 12345.67,
  "run_id": "20260608T143000-ab12cd34",
  "node": "curobo_planner_node",
  "git_commit": "b8389cf",
  "event": "curobo_plan_success",
  "data": {}
}
```

### 기록 이벤트

Perception/fusion:

- `node_start`: model, calibration, stabilization parameter
- `scene_positions_published`: 주변 ripe 과실 3D 위치
- `pick_target_rejected`: confidence/depth/mask match/stem geometry 기반 거부 사유
- `stable_pick_target_published`: 안정화된 줄기 target, quaternion, sample 수, spread,
  keypoint confidence/pixel/3D geometry/match evidence

Scan/task:

- `scan_status`: scan executor 상태 문자열
- `cell_state`: cell 상태 변경

Planning/execution:

- `pick_sequence_start`: planner가 받은 target과 시작 joints
- `pick_target_prepared`: Y clamp 및 Z bias 적용 후 target
- `collision_world_update`: cuboid와 neighbor sphere
- `curobo_plan_success`: target, 시작 joints, 전체 trajectory, latency
- `curobo_plan_fail`: 실패 status와 입력
- `curobo_plan_rejected`: joint limit/swing/spline jump 거부
- `motion_command`: MoveSplineJoint 또는 MoveLine 요청
- `motion_result`: controller 응답과 현재 joints
- `grasp_approach_complete`: 실제 사용한 offset/orientation/approach direction
- `gripper_command`: close 명령
- `marker_place_target_loaded`: tray JSON, age, slot, release/above target
- `pick_sequence_hold_latched`: preview/place/recovery 실패 후 후속 pick 차단 사유
- `pick_sequence_complete`: `SEQUENCE_COMPLETE_UNVERIFIED`

## 7. 시뮬레이션 환경에서 사용하는 방법

최소 재생 입력:

1. `node_start`에서 robot/world/parameter profile을 복원한다.
2. `scene_positions_published`와 `collision_world_update`로 scene을 구성한다.
3. `stable_pick_target_published` 또는 `pick_sequence_start`를 target으로 넣는다.
4. `curobo_plan_success.data.trajectory_rad`를 reference trajectory로 재생한다.
5. `motion_command`의 MoveLine 상대 이동을 TCP command로 재현한다.
6. simulation 결과를 원본 `motion_result` 및 실제 영상 라벨과 비교한다.

로그는 ground truth가 아니다. 특히 `pick_sequence_complete`는 실제 수확 성공으로
변환하면 안 된다.

### 로그 검사

```bash
cd ~/doosan_ws/src/e0509_gripper_description
python3 scripts/validate_runtime_jsonl.py
```

특정 파일만 검사:

```bash
python3 scripts/validate_runtime_jsonl.py logs/runtime/2026-06-08/*.jsonl
```

검사기는 JSON 파싱, 필수 field, schema version, 노드별/이벤트별 개수를 확인한다.

## 8. 시뮬레이션 통합 전 추가할 항목

- scan executor의 target ID와 root/subcell ID를 구조화 field로 전달
- 실제 controller 도착 시간 및 joint-state time series
- RGB/depth frame 또는 rosbag 경로
- 사람이 판정한 `SUCCESS`, `GRASP_EMPTY`, `DETACH_FAIL` 결과
- calibration ID, model checksum, scene ID
- simulator용 ROS topic replay 또는 rosbag2 변환 도구
