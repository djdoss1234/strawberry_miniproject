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
| SW pick 실행 시간 | 약 `36.4초` | target 수신부터 SW scan pose 복귀, Place 미포함 |
| 시간이 기록된 cuRobo 계획 시도 | 총 `5건` | 성공 `3건` + IK 실패 `2건` |
| 성공한 cuRobo 계획 | `3건` | pre-approach 후보 `2건` + scan pose 복귀 `1건` |
| 성공 계획 시간 평균 | 약 `2.68초` | 단일 run, benchmark 수치 아님 |
| 성공 계획 시간 표준편차 | 약 `2.05초` | 단일 run |
| 성공 계획 시간 최소 / 최대 | `1.17초 / 5.58초` | scan pose 복귀 계획이 최댓값 |
| 실행 전 spline-jump 거부 | `3건` | 시간이 기록된 5건과 별도인 safety guard reject |
| 관절 경로 이동 | `2건 모두 성공` | MoveSplineJoint로 pre-approach 이동 및 scan pose 복귀 |
| 그리퍼 방향 직선 이동 | `3건 모두 성공` | MoveLine으로 최종 진입 `20mm`, 추가 진입 `65mm`, 후퇴 `65mm` |
| gripper 접근 준비 상태 | position `600` | 노드 시작 및 pick 완료/실패 후 복귀 |
| 아래 방향 분리 동작 | `40mm`, 실행 성공 | MoveLine으로 BASE `-Z` 방향 당기기 |
| 최종 자동 결과 | `GRASP_UNVERIFIED` | present position read 실패 |

> PNG 상단의 `5 timed`는 성공 `3건`과 IK 실패 `2건`을 합한 값이며,
> `3 spline-jump rejects`는 실행 전에 별도로 거부된 후보 수다.
>
> planning latency는 로봇 이동 속도가 아니라, cuRobo가 계획 결과를 계산하는
> 데 걸린 **계산 소요시간**이다. **낮을수록 빠른 판단**이지만, 낮다고 성공
> 확률이 높거나 경로 품질이 좋다는 의미는 아니다.
> 이번 run에서 실패 막대가 짧은 이유는 IK 해가 없다고 비교적 빠르게 판단했기
> 때문이다.
>
> 위 planning 수치는 한 번의 SW 수확 실행에서 나온 값이다.
>
> 동일 goal 반복 100회 planner benchmark 결과는 아니다.

### 사진 6 — 로그 및 정량 결과

**삽입 위치:** 위 표 아래

**노션에 넣을 파일:**

```text
한글 설명판:
docs/runs/RUN-20260609-001_sw_runtime_summary_ko.png

원본 영문판:
docs/runs/RUN-20260609-001_sw_runtime_summary.png
```

이 이미지는 JSONL 원문을 캡처한 것이 아니라 아래 원본 로그를 읽어 생성한
요약 그래프다.

```text
원본:
logs/runtime/2026-06-09/
curobo_planner_node_20260609T160052-da5edd5a.jsonl

생성 명령:
python3 scripts/generate_runtime_summary_plot.py \
  logs/runtime/2026-06-09/curobo_planner_node_20260609T160052-da5edd5a.jsonl \
  --output docs/runs/RUN-20260609-001_sw_runtime_summary_ko.png
```

그래프 구성:

- **1번 그래프 — cuRobo Planning:** 후보별 계산 시간, 성공·실패, 실제 실행 여부
- **2번 그래프 — Robot Execution:** 선택된 모션 명령을 실제 로봇이 정상 실행했는지 확인
- **3번 그래프 — Task Timeline:** Pick 시작부터 scan pose 복귀까지 주요 단계와 전체 시간

그래프 용어:

| 용어 | 쉬운 설명 |
| --- | --- |
| 1번 계획 | 현재 SW scan pose에서 pre-approach까지 이동할 cuRobo 경로. 실제 MoveSplineJoint로 실행 |
| 2·3번 계획 | 최종 접근 endpoint 후보였으나 IK 해를 찾지 못해 실행하지 않음 |
| 4번 계획 | 최종 직선 접근 endpoint에 도달 가능한지 검증한 경로. cuRobo 경로 자체는 실행하지 않고 MoveLine 직선 접근만 실행 |
| 5번 계획 | 수확 동작 후 SW scan pose로 복귀할 cuRobo 경로. 실제 MoveSplineJoint로 실행 |
| spline-jump 거부 | 관절 각도가 `±360deg` 경계를 넘으며 크게 회전할 위험 후보를 실행 전에 차단. 수확 성공 횟수가 아니라 safety guard 동작 횟수 |
| MoveSplineJoint `2/2 성공` | 1번 pre-approach 경로와 5번 scan pose 복귀 경로를 실제 로봇이 정상 실행 |
| TOOL MoveLine `3/3 성공` | 4번에서 endpoint 가능성을 확인한 뒤 최종 진입, 추가 진입, 후퇴를 직선으로 실행 |
| BASE detach pull `1/1 성공` | 로봇 베이스 좌표 기준 아래 방향으로 `40mm` 당기는 분리 동작을 정상 실행 |

여기서 `성공`은 해당 **모션 명령 실행 성공**을 의미한다. 실제 줄기 파지나
최종 수확 성공은 7번의 영상 판정 기준으로 별도 확인한다.

JSONL 원문의 긴 텍스트 화면은 노션 본문에 넣지 않는다. 필요하면 증거용으로
`verify_grasp`, `verify_detach`, `pick_sequence_complete` 세 이벤트만 접어서
첨부한다.

**권장 캡션:**

`실제 로봇 실행을 JSONL로 기록하여 계획 시간, reject 원인, motion 결과와 전체 cycle time을 추적한다.`

---

## 7. SW 수확 성공 판정 기준

이번 SW 실험을 통해 **모션 완료와 실제 수확 성공을 분리하여 기록해야 한다**는
기준을 확정했다.

```text
줄기 파지 성공
 -> 줄기에서 과실 분리 성공
 -> retreat 후에도 과실 유지
 -> 최종 수확 성공
```

| 판정 단계 | 성공 기준 | 현재 확인 방법 |
| --- | --- | --- |
| 줄기 파지 성공 | 그리퍼가 잎이나 과실 몸체가 아닌 줄기를 정확히 잡음 | 실험 영상 육안 판정 |
| 과실 분리 성공 | BASE `-Z` detach pull 후 과실이 줄기에서 분리됨 | 실험 영상 육안 판정 |
| 후퇴 유지 성공 | retreat 완료까지 과실을 놓치지 않음 | 실험 영상 육안 판정 |
| 최종 수확 성공 | 파지·분리·후퇴 유지가 모두 성공 | 위 세 판정을 모두 만족 |

현재 코드와 JSONL은 다음 값을 자동으로 기록한다.

| 자동 로그 | 의미 |
| --- | --- |
| `grasp_pose_reached` | 계획된 파지 위치까지 모션 실행 완료 |
| `GRASP_CONTACT_DETECTED` | 그리퍼가 `665` 미만에서 멈춰 무언가 잡혔다고 추정 |
| `GRASP_EMPTY` | 그리퍼가 `665` 이상까지 닫혀 빈 파지라고 추정 |
| `GRASP_UNVERIFIED` | 그리퍼 위치값 읽기 실패로 자동 판정 불가 |
| `detach_pull_down` | BASE `-Z 40mm` 분리 동작 실행 |
| `pick_sequence_complete` | scan pose 복귀까지 시퀀스 완료 |

`grasp_pose_reached`와 `pick_sequence_complete`는 실제 수확 성공을 의미하지
않는다. 또한 그리퍼 위치값은 잎을 잡은 경우도 접촉으로 오인할 수 있으므로,
현재 최종 성공 판정은 영상을 기준으로 한다.

이번 SW 작업에서는 줄기 파지와 과실 분리 성공 사례를 육안으로 확인했다.
하지만 반복 횟수가 부족하고 자동 파지 판독이 `GRASP_UNVERIFIED`인 경우가 있어,
아직 수확 성공률을 정량 수치로 제시하지 않는다.

---

## 8. SW 수확 정량 측정 지표 및 후속 계획

현재 SW 페이지에서는 이번 작업의 결과를 평가하는 데 필요한 핵심 지표만
관리한다.

| 핵심 지표 | 계산 방법 | 현재 상태 |
| --- | --- | --- |
| 줄기 파지 성공률 | 줄기를 정확히 잡은 횟수 / 전체 close 시도 | 반복 영상 라벨 필요 |
| 최종 수확 성공률 | 파지·분리·후퇴 유지 성공 / 전체 시도 | 반복 영상 라벨 필요 |
| 평균 수확 시간 | target 확정부터 scan pose 복귀까지 평균 시간 | 단일 run `36.4초`, 반복 측정 필요 |
| 모션 구간 완료율 | 계획·MoveSplineJoint·MoveLine 구간을 모두 완료한 시도 / 전체 시도 | JSONL로 계산 가능 |
| 비목표 접촉률 | 잎·다른 과실·보드와 접촉한 접근 / 전체 접근 시도 | 반복 영상 라벨 필요 |
| 사람 개입률 | 수동 복구 또는 재시작이 필요했던 시도 / 전체 시도 | 반복 실험부터 기록 |

모형 딸기를 사용하므로 실제 과실 손상률은 현재 지표에서 제외한다.

### 다음 실험에서 추가할 지표

| 후속 작업 | 추가 측정 항목 |
| --- | --- |
| SW 최소 30회 반복 | 위 6개 핵심 지표의 평균·비율 확정 |
| marker place 검증 | Place 성공률, tray pose 오차, slot center 오차 |
| NE 군집 / NW 가림 셀 검증 | 셀 난이도별 수확 성공률, 충돌 및 실패 원인 |
| planner 비교 실험 | 계획 시간, 계획 성공률, reject 원인, 경로 길이, jerk |

Place와 세부 플래너 성능은 아직 이번 SW 단일딸기 수확 결과가 아니므로, 현재
성과 지표에 섞지 않고 후속 검증 결과에서 별도로 정리한다.

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
