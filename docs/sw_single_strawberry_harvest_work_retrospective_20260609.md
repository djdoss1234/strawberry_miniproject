# SW셀 단일 딸기 수확 작업 회고

작성일: 2026-06-09

## 프로젝트 배경

### 해결하려 한 문제

SW 셀은 단일 딸기로 구성된 가장 쉬운 수확 조건이지만, 화면에서 딸기를
검출하는 것만으로는 실제 수확이 되지 않았다. 검출 좌표가 흔들리거나 cuRobo가
다른 IK branch를 선택하고, 최종 TCP 이동이 곡선이 되면 15.8cm 연장 파츠가
잎과 과실을 밀었다. 줄기를 잡아도 뒤로 빠지는 동작만으로는 분리되지 않았다.

### 왜 중요한가

농업 자동화에서는 검출 성공보다 실제 과실을 손상 없이 분리하고 다음 작업을
이어가는 것이 중요하다. 단일 과실에서도 접근, 파지, 분리, 복귀를 재현하지
못하면 군집 과실과 가림 환경으로 확장할 수 없다. 따라서 SW셀을 전체 수확
시스템의 최소 검증 단위로 정의했다.

## 문제 정의

민석은 문제를 단순한 `IK 실패`가 아니라 네 단계로 분해했다.

1. **Perception:** 줄기 target의 깊이와 높이 오차
2. **Planning:** IK branch, 관절 경계, 잎이 없는 불완전 collision world
3. **Execution:** joint spline이 만드는 측방 진입과 실제 TCP 직선성
4. **Verification:** `pick_complete`와 실제 수확 성공의 불일치

cuRobo만으로 scan pose부터 줄기까지 이동하면 빠르지만 최종 TCP 직선성을
보장하지 못했다. 반대로 모든 이동을 수동 티칭하면 환경 변화에 대응하기
어렵다. 그래서 cuRobo는 긴 구간의 실행 가능성과 관절 branch를 검증하고,
줄기 근처에서는 Doosan MoveLine으로 정지 후 직선 진입하는 hybrid 정책을
선택했다.

## 해결 과정

### 기술 스택과 선택 이유

- **YOLO segmentation + pose + RGB-D:** ripe 상태와 줄기 keypoint를 함께 얻고
  `base_link` 기준 3D target을 만들기 위해 사용했다.
- **cuRobo MotionGen:** scan pose에서 pre-approach까지 IK, 관절 branch,
  whiteboard 및 이웃 과실 충돌을 검사하기 위해 사용했다.
- **Doosan MoveLine:** 마지막 줄기 접근을 TOOL `+Z` 직선으로 고정하기 위해
  사용했다.
- **BASE `-Z` detach pull:** 정면 역진만으로 분리되지 않는 문제를 아래 방향
  `40mm` 당김으로 해결하기 위해 사용했다.
- **JSONL + Git:** 파라미터 변경과 실행 결과를 연결해 재현하기 위해 사용했다.

### 문제와 직접 내린 결정

| 문제 | 원인 분석 | 결정 및 수정 |
| --- | --- | --- |
| 아래에서 위로 접근 | orientation 접근축에 약 `+14.7deg` 상승 성분 | 수평 orientation과 pitch 후보 탐색 |
| 크게 회전하거나 옆으로 진입 | cuRobo joint trajectory가 TCP 직선을 보장하지 않음 | pre-approach 정지 후 MoveLine 직선 진입 |
| 줄기보다 낮거나 얕게 접근 | target/TCP 모델과 실물 위치 차이 | grasp Z `+30mm`, 추가 진입 `65mm` |
| 1-step 최적화 후 정확도 저하 | 긴 spline이 줄기 근처까지 담당 | 2-step pre-approach 구조 복원 |
| 파지 후 미분리 | 정면 역진만으로 분리력 부족 | BASE `-Z 40mm` detach pull |
| 코드가 성공 여부를 모름 | gripper hardware read 실패 | `GRASP_UNVERIFIED`와 육안 판정 분리 |

### AI 활용

Claude Code와 Codex는 ROS 로그 비교, 코드 경로 탐색, IK branch 및 파라미터
가설 정리에 활용했다. 어떤 정책을 실기에 적용할지, 어떤 값을 유지하거나
폐기할지, 안전하게 다음 테스트로 넘어갈지는 민석이 실제 로봇과 과실 접촉을
관찰하여 최종 결정했다.

## 성과

| 항목 | Before | After |
| --- | --- | --- |
| SW 단일 딸기 수확 | 접근 실패, 측방 진입, 얕은 진입, 미분리 반복 | 육안 기준 줄기 파지·분리 성공 사례 확보 |
| 최종 접근 | cuRobo spline 중심 | 정지 후 TOOL `+Z` 직선 진입 |
| 줄기 목표 높이 | 보정 없음/불충분 | Z bias `+30mm` |
| 추가 진입 | 부족하거나 실험별 변동 | 현재 `65mm` |
| 분리 동작 | 정면 역진 중심 | BASE `-Z 40mm` pull |
| 최신 완료 시퀀스 | 기준 측정 없음 | 약 `36.4초` |
| 자동 수확 성공률 | 측정 불가 | **측정 필요** (`GRASP_UNVERIFIED`) |
| 3D target 오차 | 측정 없음 | **측정 필요** |
| 손상률 / drop rate | 측정 없음 | **측정 필요** |

수확 성공률은 `그리퍼 접촉 -> 줄기 분리 -> retreat 후 유지`가 모두 확인된
시도만 성공으로 집계한다. 동일 SW 조건에서 최소 30회를 수행하고,
`GRASP_EMPTY`, `GRASP_UNVERIFIED`, `DETACH_FAIL`, `DROP_DURING_RETREAT`,
`SUCCESS`를 각각 기록해야 한다.

실사용 관점에서의 성과는 한 번 딸기를 딴 것뿐 아니라, 실패를 perception,
planning, execution, verification 단계로 구분하고 재현 가능한 로그를 만든
것이다. 이는 농가 환경에서 작업자 개입 원인을 줄이고 자동 수확을 포기해야
하는 조건을 설명하는 기반이 된다.

## 자소서 소재 메모

### 드러난 역량

- 실패를 단계별 원인과 로그로 분리하는 구조적 문제 해결 습관
- 마지막 접촉 구간에 결정적인 직선 동작을 배치하는 판단력
- 라이브러리 성공 로그보다 실제 하드웨어 관찰을 우선하는 검증 태도
- 단일 과실부터 성공 조건을 만든 뒤 복잡 셀로 확장하는 단계적 접근

### 면접에서 “어려웠던 점은?”이라고 물으면

> 가장 어려웠던 점은 planner 로그상 성공과 실제 수확 성공이 달랐다는
> 점입니다. cuRobo는 유효한 joint trajectory를 만들었지만 줄기 근처에서 TCP가
> 곡선으로 접근하며 잎과 과실을 밀었습니다. 저는 이를 planner 실패로만 보지
> 않고 최종 접촉 구간의 실행 방식 문제로 정의했습니다. cuRobo는 pre-approach와
> 안전성 검증에 사용하고 마지막 구간은 MoveLine 직선 진입으로 분리했습니다.
> 이후 BASE -Z 40mm 분리 동작과 JSONL 로그를 추가해 SW 단일 과실의 육안
> 수확 성공 사례를 확보했습니다.

### 한 줄 자소서

> AI 모션 플래너의 성공 로그를 그대로 믿지 않고 실제 로봇의 접촉 실패를
> 계층별로 분석해 수확 가능한 hybrid motion pipeline으로 개선했습니다.

## 향후 기록

- 동일 SW 조건 최소 30회 반복 성공률, cycle time, 손상률 측정
- planning latency와 대기 시간을 분리한 시간 단축
- marker 기반 place 검증
- NE 군집, NW 가림 셀 수확
- 당시 감정: `[민석이 반복 실패와 첫 실제 분리 성공 당시 느낀 점 추가]`
