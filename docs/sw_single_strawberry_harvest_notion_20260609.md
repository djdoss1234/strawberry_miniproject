# SW셀(단일딸기) 수확 모션 구현 및 실기 검증

## 1. 배경 및 목적

이전 작업에서는 카메라 중심이 아닌 **그리퍼가 수확하기 좋은 방향**을 기준으로
4개 셀의 스캔 포즈를 다시 티칭했다.

다음 단계는 가장 단순한 SW 단일딸기 셀에서 실제 수확 동작을 검증하는 것이었다.

이번 작업의 목표:

```text
SW scan pose
 -> 정상 딸기와 줄기 인식
 -> 줄기까지 접근
 -> 그리퍼 파지
 -> 줄기에서 과실 분리
 -> SW scan pose 복귀
```

SW셀부터 시작한 이유:

| 셀 | 환경 구성 | 역할 |
| --- | --- | --- |
| SW | 단일 딸기 | 기본 수확 동작 검증 |
| NE | 군집 딸기 | 이웃 과실 충돌 및 target 선택 검증 예정 |
| NW | 잎/줄기 가림 | 재관측 및 접근 불가 판단 검증 예정 |
| SE | 빈 셀 | empty 판단 검증 예정 |

> **현재 범위:** SW 단일딸기의 접근·파지·분리·복귀까지 검증했다.
>
> **다음 작업:** 이 페이지 작성 후 marker 기반 계란판 place를 진행한다.

### 사진 1 — SW 셀 환경

**삽입 위치:** 이 문단 바로 아래

**권장 사진:** `Screenshot from 2026-06-02 17-40-19.png`

**사진에서 보여줄 내용:**

- SW 단일딸기 환경
- 그리퍼 파츠가 셀 중심 방향을 향하는 모습
- 다른 셀보다 단순하여 첫 수확 검증 대상으로 선택한 이유

**권장 캡션:**

`SW 단일딸기 셀의 gripper-centered scan view. 기본 수확 모션을 검증하기 위한 최소 난이도 환경이다.`

---

## 2. 핵심 문제 — 인식 성공이 실제 수확 성공은 아니었다

카메라에서는 딸기와 줄기 target이 검출됐지만 실제 로봇 수확은 반복적으로
실패했다.

| 문제 | 실제 현상 |
| --- | --- |
| 접근 방향 오류 | 그리퍼가 아래에서 위로 올라가거나 옆으로 크게 회전함 |
| TCP 경로 오류 | 목표 좌표는 맞아도 접근 중 잎과 딸기를 밀어냄 |
| 진입 깊이 부족 | 줄기 앞에서 멈춰 그리퍼가 줄기를 감싸지 못함 |
| 목표 높이 오류 | 줄기보다 아래를 지나며 잎이나 과실 넓은 부분과 접촉함 |
| 분리 실패 | 줄기를 잡아도 정면으로 후퇴하면 딸기가 분리되지 않음 |
| 성공 판정 불일치 | `grasp OK`, `pick_complete`가 출력돼도 실제 수확은 실패할 수 있음 |

따라서 문제를 다음 네 단계로 분리했다.

```text
Perception
 -> 줄기 target이 정확한가?

Planning
 -> 해당 target까지 도달 가능한 IK/경로인가?

Execution
 -> 실제 TCP가 원하는 방향으로 접근했는가?

Verification
 -> 실제로 잡고, 분리하고, 후퇴 중 유지했는가?
```

### 사진 2 — 비전 인식 화면

**삽입 위치:** 위 문제 분류 표 아래

**권장 사진:** `Screenshot from 2026-06-04 14-25-01.png`

**사진에서 보여줄 내용:**

- segmentation과 pose keypoint가 동시에 표시되는 화면
- 화면상 검출 성공과 실제 파지 성공은 별개라는 점
- 잎/과실 가림과 target 흔들림

**권장 캡션:**

`Segmentation과 줄기 keypoint가 검출돼도, target 오차와 실제 접근 경로 때문에 수확은 실패할 수 있다.`

---

## 3. 시스템 파이프라인

```text
RealSense RGB-D
 -> YOLO segmentation + pose fusion
 -> ripe 필터 + 줄기 keypoint 안정화
 -> depth + eye-in-hand calibration + E0509 FK
 -> base_link 기준 줄기 target 생성
 -> 그리퍼를 approach position 600으로 유지
 -> scan_executor가 현재 셀 target 하나 전달
 -> cuRobo가 pre-approach와 grasp endpoint 검증
 -> MoveSplineJoint로 pre-approach 이동
 -> 정지 후 MoveLine TOOL +Z 직선 진입
 -> RH-P12-RN-A gripper position 700으로 close
 -> MoveLine BASE -Z 40mm detach pull
 -> MoveLine TOOL -Z 직선 retreat
 -> cuRobo + MoveSplineJoint로 SW scan pose 복귀
 -> 다음 수확을 위해 gripper position 600으로 복귀
 -> runtime JSONL에 결과 기록
```

### 각 기술의 역할

| 기술 | 적용 구간 | 선택 이유 |
| --- | --- | --- |
| YOLO seg + pose | 숙도 및 줄기 후보 검출 | 익은 과실만 선택하고 줄기 접근점을 얻기 위해 |
| RGB-D + hand-eye + FK | 2D target을 `base_link` 3D 좌표로 변환 | 실제 로봇 목표 좌표를 만들기 위해 |
| cuRobo MotionGen | pre-approach 및 endpoint 검증 | IK, 관절 branch, whiteboard/이웃 과실 충돌을 검사하기 위해 |
| MoveSplineJoint | cuRobo joint trajectory 실행 | 여러 joint waypoint를 부드럽게 실제 로봇에서 실행하기 위해 |
| MoveLine TOOL `+Z` | 최종 줄기 접근 | 줄기 근처에서 TCP 직선 진입을 보장하기 위해 |
| MoveLine BASE `-Z` | 과실 분리 | 정면 후퇴 대신 아래 방향으로 당겨 줄기를 분리하기 위해 |
| Gripper position `600 -> 700 -> 600` | 접근 준비, 파지, 다음 사이클 준비 | 접근 중 파츠 사이에 줄기가 들어올 여유를 유지하고 다음 수확 시 걸림을 방지하기 위해 |
| JSONL runtime log | 전체 실행 과정 | 성공/실패 원인을 재현하고 추후 시뮬레이션에 사용하기 위해 |

---

## 4. 왜 cuRobo만 사용하지 않고 Hybrid Motion을 사용했나

초기에는 cuRobo가 생성한 joint trajectory를 이용해 scan pose부터 grasp
위치까지 접근했다.

cuRobo는 목표 위치까지 도달 가능한 joint 경로를 생성할 수 있지만, 줄기 근처
TCP가 완전한 직선으로 움직인다는 보장은 없다.

실제 실험에서는 다음 문제가 발생했다.

```text
cuRobo 경로 생성 성공
 -> MoveSplineJoint 실행 성공
 -> 하지만 TCP가 곡선으로 접근
 -> 그리퍼 파츠가 잎/딸기와 접촉
 -> 실제 파지 실패
```

따라서 역할을 분리했다.

```text
긴 이동 및 안전성 검증
 = cuRobo + MoveSplineJoint

줄기 근처 최종 접근 및 후퇴
 = Doosan MoveLine
```

이 구조를 통해 planner의 유연성과 마지막 접촉 구간의 직선성을 동시에 확보했다.

### 사진/영상 3 — Hybrid Motion 동작 비교

**삽입 위치:** 이 절 마지막

**필요 자료:** 새로 캡처 필요

**권장 구성:** 좌우 비교 영상 또는 GIF

- 왼쪽: cuRobo spline이 줄기 근처까지 접근하며 측방 이동하는 실패 장면
- 오른쪽: pre-approach에서 정지 후 MoveLine으로 정면 직선 진입하는 장면

**권장 캡션:**

`cuRobo는 pre-approach까지 사용하고, 실제 접촉 구간은 MoveLine으로 분리하여 측방 접근을 줄였다.`

---

## 5. 개발 및 트러블슈팅 과정

### STEP 1 — 그리퍼 Approach Position 600 상태 적용

**최초 구현일: 2026-06-05**

초기에는 pick이 끝난 뒤 그리퍼가 close position `700` 상태로 남아 다음 scan 및
접근을 시작했다.

이 상태에서는 다음 줄기가 파츠 사이로 들어올 공간이 부족하고, 닫힌 파츠가
잎이나 줄기에 걸릴 수 있었다. 또한 pick 시작마다 다시 open 명령을 보내면
불필요한 대기와 동작이 추가됐다.

따라서 그리퍼 상태를 다음처럼 변경했다.

```text
노드 시작 2초 후
 -> position 600 자동 설정

scan / pre-approach / final approach
 -> position 600 유지

grasp pose 도착
 -> position 700 close

pick 완료 또는 실패
 -> position 600 복귀
```

| 상태 | Position | 의미 |
| --- | --- | --- |
| 접근 준비 | `600` | 줄기가 파츠 사이로 들어올 수 있는 개도 유지 |
| 파지 | `700` | 줄기 파지 close 명령 |
| 완료/실패 후 | `600` | 다음 scan 및 pick 준비 |

현재 position `600`은 줄기만 파츠 사이에 들어오면 적절한 접근 개도다. 다만
실제 contact/force 값이 아니라 명령 position이므로, 파지 성공 판정과는 별도로
관리한다.

관련 커밋:

```text
37fef71  2026-06-05  OPEN_GRIPPER_ON_PICK_START=False
bdd04f4  2026-06-05  노드 시작/pick 완료·실패 후 gripper 600 복귀
```

### 사진/영상 4 — 그리퍼 600 접근 상태

**삽입 위치:** 이 절 마지막

**필요 자료:** 새로 촬영 필요

**권장 구성:** 같은 카메라 각도에서 두 장 비교

- position `600`: 줄기가 파츠 사이로 들어올 수 있는 접근 상태
- position `700`: 파지 close 상태

**권장 캡션:**

`접근 중에는 position 600으로 줄기 진입 공간을 유지하고, grasp pose에서만 position 700으로 닫는다.`

---

### STEP 2 — 접근 orientation 수정

기존 orientation의 접근 방향에는 약 `+14.7deg` 상승 성분이 포함되어 있었다.

이 때문에 그리퍼가 줄기 정면이 아니라 아래에서 위로 접근했다.

수평 orientation과 pitch 후보를 순차적으로 검증하여, 최종 접근 방향이
화이트보드 정면을 향하도록 수정했다.

---

### STEP 3 — Stop-Then-Straight 접근 적용

1-step cuRobo 접근은 planning 단계를 줄일 수 있었지만, 긴 joint spline이 줄기
근처까지 이어지면서 최종 접근 정확도가 떨어졌다.

따라서 2-step 구조를 사용했다.

```text
1. cuRobo + MoveSplineJoint로 pre-approach 이동
2. 완전히 정지
3. MoveLine TOOL +Z로 줄기까지 직선 진입
```

현재 pre-approach는 `60mm` 설정을 재검증 중이다.

---

### STEP 4 — 줄기 target 높이 및 진입 깊이 보정

그리퍼 파츠가 줄기보다 아래를 지나면서 잎과 과실을 밀어내는 문제가 있었다.

수정값:

| 항목 | 현재 값 | 목적 |
| --- | --- | --- |
| grasp Z bias | `+30mm` | 줄기보다 아래로 접근하는 현상 보정 |
| extra advance | `65mm` | 줄기 앞에서 멈추는 진입 깊이 부족 보정 |
| final approach velocity | `50mm/s` | 최종 접근 속도 제어 |

---

### STEP 5 — 파지 후 분리 동작 변경

초기에는 파지 후 진입 경로를 정면으로 역주행했다.

하지만 줄기를 잡은 상태에서도 딸기가 분리되지 않는 문제가 발생했다.

따라서 먼저 아래 방향으로 당긴 뒤 후퇴하도록 변경했다.

```text
gripper close
 -> BASE -Z 40mm detach pull
 -> TOOL -Z retreat
```

현재 detach pull:

| 항목 | 값 |
| --- | --- |
| 방향 | BASE `-Z` |
| 거리 | `40mm` |
| 속도 | `50mm/s` |

---

### STEP 6 — 관절 branch 및 spline jump 차단

J4 등가각과 IK branch가 달라지면 로봇이 목표 근처에서 크게 회전하는 문제가
발생했다.

cuRobo 경로 생성 후 다음 항목을 추가 검사했다.

- operational joint limit
- 큰 joint swing
- `±360deg` 경계에서 발생하는 spline jump
- 실행 전 grasp endpoint IK 가능 여부

유효하지 않은 후보는 실행하지 않고 다음 orientation/offset 후보를 탐색한다.

---

### STEP 7 — SW 단일딸기 파지 및 분리 관찰

반복 실기 검증을 통해 SW 단일딸기에서 줄기 파지와 과실 분리 성공 사례를
육안으로 확인했다.

단, 현재 그리퍼 상태 읽기가 실패하는 경우가 있어 자동 결과는
`GRASP_UNVERIFIED`로 기록한다.

### 사진/영상 5 — SW 수확 성공 증거

**삽입 위치:** 이 절 바로 아래

**필요 자료:** 새로 캡처 필요

**반드시 필요한 장면:**

1. 파지 직전 줄기와 열린 그리퍼
2. 줄기를 잡은 상태
3. BASE `-Z` detach 후 과실이 줄기에서 분리된 상태
4. retreat 후에도 과실이 그리퍼에 유지된 상태

**권장 방식:** 한 영상에서 위 네 장면을 캡처해 4분할 이미지로 제작

**권장 캡션:**

`SW 단일딸기에서 접근, 줄기 파지, 아래 방향 분리, retreat 유지까지 육안 확인한 과정.`

---

## 6. 현재까지 확인된 정량 결과

기준 runtime log:

```text
logs/runtime/2026-06-09/
curobo_planner_node_20260609T160052-da5edd5a.jsonl
```

### 최신 완료 run 측정값

| 항목 | 측정값 | 비고 |
| --- | --- | --- |
| 전체 pick cycle time | 약 `36.4초` | target 수신부터 SW scan pose 복귀 |
| 성공한 cuRobo 계획 수 | `3건` | 한 번의 수확 중 후보 탐색 및 복귀 계획 |
| 성공 계획 시간 평균 | 약 `2.68초` | 단일 run, benchmark 수치 아님 |
| 성공 계획 시간 표준편차 | 약 `2.05초` | 단일 run |
| 성공 계획 시간 최소 / 최대 | `1.17초 / 5.58초` | 복귀 joint-space 계획이 최댓값 |
| 실패한 IK 계획 | `2건` | 다음 후보 탐색 후 진행 |
| spline jump reject | `3건` | 위험 branch 실행 전 차단 |
| MoveSplineJoint 실행 | `2건 모두 success` | pre-approach, scan pose 복귀 |
| MoveLine 실행 | `3건 모두 success` | final approach, extra advance, retreat |
| gripper 접근 준비 상태 | position `600` | 노드 시작 및 pick 완료/실패 후 복귀 |
| extra advance | `65mm` | TOOL `+Z` |
| detach pull | `40mm` | BASE `-Z` |
| 최종 자동 결과 | `GRASP_UNVERIFIED` | present position read 실패 |

> 위 planning 수치는 한 번의 SW 수확 실행에서 나온 값이다.
>
> 동일 goal 반복 100회 planner benchmark 결과는 아니다.

### 사진 6 — 로그 및 정량 결과

**삽입 위치:** 위 표 아래

**필요 자료:** 새로 캡처 또는 그래프 생성 필요

**권장 자료:**

- runtime JSONL의 `curobo_plan_success`, `curobo_plan_fail`,
  `pick_sequence_complete` 이벤트 캡처
- planning latency 막대그래프
- 전체 `36.4초` 시퀀스 타임라인

**권장 캡션:**

`실제 로봇 실행을 JSONL로 기록하여 계획 시간, reject 원인, motion 결과와 전체 cycle time을 추적한다.`

---

## 7. 파지 성공 정량 판정 기준

`grasp OK`는 그리퍼가 목표 위치에 도달했다는 뜻이며, 실제 딸기 파지 성공을
의미하지 않는다.

파지 성공은 다음 세 단계를 모두 확인해야 한다.

```text
실제 접촉
 -> 줄기에서 과실 분리
 -> retreat 후에도 과실 유지
 -> End-to-End Harvest Success
```

| 지표 | 계산식 / 판정 기준 | 현재 상태 |
| --- | --- | --- |
| Grasp verification coverage | 유효 gripper position 판독 / close 시도 | hardware read 안정화 필요 |
| Contact detection rate | `GRASP_CONTACT_DETECTED` / 유효 판독 | position `< 665` 기준 구현 |
| Empty grasp rate | `GRASP_EMPTY` / 유효 판독 | position `>= 665` 기준 구현 |
| Detach success rate | 줄기에서 분리된 과실 / 파지 시도 | 현재 육안 판정 |
| Retention success rate | retreat 후 유지된 과실 / 분리 성공 | 측정 필요 |
| End-to-end harvest success | 접촉+분리+유지 성공 / 전체 시도 | 측정 필요 |
| Grasp verifier precision/recall | 자동 판정과 사람 라벨 비교 | 측정 필요 |

현재 jaw position 판정은 그리퍼 사이에 무언가가 끼었는지를 간접 판단한다.
잎을 잡아도 접촉 성공으로 오인할 수 있으므로, position 값만으로 최종 수확
성공을 선언하지 않는다.

---

## 8. 다음 place 및 복잡 셀 검증에서 측정할 지표

현재 수치는 SW 수확 모션을 안정화하는 과정에서 확보한 단일 run 기준이다.
다음 place 및 NE/NW 검증부터 동일 형식으로 반복 데이터를 수집한다.

### Planner 성능

| 지표 | 측정 계획 |
| --- | --- |
| 계획 시간 평균/표준편차/최악값 | 동일 start/goal offline 또는 simulation 100회 |
| 계획 성공률 | 동일 조건 100회 계획 |
| joint/cartesian 경로 길이 | trajectory 및 FK 기반 계산 |
| 재계획 빈도 | 이웃 과실 및 동적 장애물 갱신 시 기록 |

### 경로 품질

| 지표 | 측정 계획 |
| --- | --- |
| velocity/acceleration/jerk | trajectory timestamp 기반 계산 |
| 실제 실행 시간 | motion command부터 motion result까지 |
| 목표점 도달 오차 | FK 기반 오차 + marker/외부 측정 기준 추가 |
| 충돌 회피 성공률 | 시나리오별 충돌 없는 계획 및 실제 실행 비율 |

### 수확 및 Place

| 지표 | 측정 계획 |
| --- | --- |
| SW 반복 수확 성공률 | 동일 조건 최소 30회 |
| grasp/detach/retention 성공률 | 단계별 result code와 영상 라벨 비교 |
| fruit damage/drop rate | 실험 후 사람 라벨 및 이미지 기록 |
| tray pose / slot center 오차 | marker localization 결과와 실제 hole center 비교 |
| place success / drop rate | fresh tray localization 기반 반복 검증 |
| 전체 cycle time | scan -> pick -> place -> 다음 target까지 측정 |

> **측정 예정 시점:** 이 페이지 작성 후 진행할 marker place 검증과 NE/NW 복잡
> 셀 수확 검증부터 반복 KPI를 수집한다.

---

## 9. 현재 결론

SW 단일딸기 실험을 통해 다음을 확인했다.

1. 비전 인식 성공만으로 실제 수확 성공을 보장할 수 없다.
2. cuRobo는 긴 이동과 실행 가능성 검증에 적합하지만, 줄기 근처 최종 TCP
   직선 접근은 MoveLine으로 분리하는 것이 효과적이었다.
3. 파지 후 정면 후퇴보다 BASE `-Z 40mm` detach pull이 실제 과실 분리에
   적합했다.
4. planner 성공, gripper close, 실제 수확 성공을 서로 다른 결과로 기록해야
   한다.
5. SW 육안 수확 성공 사례는 확보했지만, 자동 검증된 성공률은 아직 측정하지
   않았다.

현재 단계:

```text
SW 단일딸기 수확 모션 검증
 -> 육안 성공 사례 확보
 -> runtime 정량 로그 구조 확보
 -> 반복 KPI 측정 필요
 -> marker place 검증 진행 예정
 -> NE 군집 / NW 가림 셀 확장 예정
```

---

## 10. 관련 파일 및 로그

```text
scripts/strawberry_fusion_node.py
scripts/curobo_planner_node.py
config/environment.yaml
docs/runtime_pipeline_and_simulation_logs.md
docs/harvest_motion_session_20260607.md
docs/harvest_motion_session_20260609.md
docs/experiment_results.md
logs/runtime/2026-06-09/
docs/runs/RUN-20260607-001_sw_horizontal_straight_approach.log
```

### 마지막 사진/영상 — 다음 단계 연결

**삽입 위치:** 페이지 맨 아래

**권장 사진:** `Screenshot from 2026-06-08 16-37-45.png`

**사진에서 보여줄 내용:**

- 수확 후 계란판을 바라보는 카메라 화면
- marker 기반 tray localization 및 place가 다음 단계인 이유

**주의:** 이 사진은 place 성공 증거가 아니다. 현재는 계란판 view 및 marker
인식 검증 참고 이미지로만 사용한다.

**권장 캡션:**

`SW 수확 이후 다음 단계는 이동 가능한 계란판의 marker localization과 자동 place 검증이다.`
