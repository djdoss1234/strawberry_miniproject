# SW셀 단일 딸기 수확 회고 및 포트폴리오 소재

작성일: 2026-06-09

## 근거 수준

- **구현 확인:** RGB-D 인식부터 SW scan pose 복귀까지의 수확 시퀀스가 코드와
  runtime JSONL에 기록된다.
- **실험 관찰:** 민석이 SW 단일 딸기의 줄기 파지 및 분리 성공 사례를 육안으로
  확인했다.
- **자동 판정 한계:** 그리퍼 상태 읽기가 실패하여 결과 코드는 아직
  `GRASP_UNVERIFIED`다. 반복 성공률, 과실 손상률, 3D 오차는 측정 필요다.
- **현재 범위:** place와 NE 군집/NW 가림 셀의 수확은 완료로 주장하지 않는다.

## 프로젝트 배경

### 해결하려 한 문제

화이트보드의 SW 셀은 단일 딸기로 구성되어 있어 가장 쉬운 수확 조건이지만,
화면에서 딸기를 검출하는 것만으로는 실제 수확이 되지 않았다. 검출 좌표가
조금만 흔들리거나, cuRobo가 다른 IK branch를 선택하거나, 최종 TCP 이동이
곡선이 되면 15.8cm 연장 파츠가 잎과 과실을 밀었다. 줄기를 잡아도 뒤로만
빠지면 분리되지 않는 문제도 있었다.

### 왜 중요한가

농업 자동화에서는 검출 정확도보다 실제 과실을 손상 없이 분리하고 다음 작업을
이어갈 수 있는지가 더 중요하다. 가장 단순한 단일 과실에서도 접근, 파지, 분리,
복귀를 재현하지 못하면 군집 과실이나 가림 환경으로 확장할 수 없다. 따라서
SW셀은 전체 수확 시스템의 최소 검증 단위로 정의했다.

## 문제 정의

민석은 문제를 단순한 `IK 실패`가 아니라 다음 네 층으로 분해했다.

1. **Perception:** 줄기 target의 깊이와 높이 오차
2. **Planning:** IK branch, 관절 경계, 잎이 없는 불완전 collision world
3. **Execution:** joint spline이 만드는 측방 진입과 실제 TCP 직선성
4. **Verification:** `pick_complete`와 실제 수확 성공의 불일치

cuRobo만으로 scan pose부터 줄기까지 한 번에 이동하는 방법은 빠르지만 최종 TCP
직선성을 보장하지 못했다. 반대로 모든 이동을 수동 티칭하면 환경 변화에
대응하기 어렵다. 그래서 **cuRobo는 긴 구간의 실행 가능성과 관절 branch를
검증하고, 줄기 근처에서는 Doosan MoveLine으로 정지 후 직선 진입하는 hybrid
정책**을 선택했다.

## 해결 과정

### 기술 선택과 이유

- **YOLO segmentation + pose + RGB-D:** ripe 상태와 줄기 keypoint를 함께 얻고,
  카메라 픽셀이 아닌 `base_link` 3D target으로 변환하기 위해 사용했다.
- **cuRobo MotionGen:** scan pose에서 pre-approach까지 IK, 관절 branch,
  whiteboard/이웃 과실 충돌을 함께 검사하기 위해 사용했다.
- **Doosan MoveLine:** 마지막 줄기 접근을 관절 spline이 아닌 TOOL `+Z` 직선으로
  고정하기 위해 사용했다.
- **BASE `-Z` detach pull:** 파지 후 단순 역진만으로 분리되지 않는 문제를
  결정적인 아래 방향 `40mm` 당김으로 바꾸기 위해 사용했다.
- **JSONL runtime log + Git commit:** 육안 느낌과 코드 변경을 분리하고, 어떤
  파라미터가 어떤 실행 결과를 만들었는지 재현하기 위해 사용했다.

### 막혔던 지점과 원인

| 막힌 지점 | 원인 분석 | 결정 및 수정 |
| --- | --- | --- |
| 그리퍼가 아래에서 위로 접근 | 기존 orientation의 접근축이 약 `+14.7deg` 상승 | 수평 orientation 및 pitch 후보 탐색 |
| 접근 중 크게 회전하거나 옆으로 진입 | cuRobo joint trajectory가 TCP 직선을 보장하지 않음 | pre-approach에서 정지 후 MoveLine 직선 진입 |
| 줄기보다 낮거나 얕게 접근 | target/TCP 모델 오차와 실제 줄기 위치 차이 | grasp Z `+30mm`, 추가 진입 `65mm` 적용 |
| 1-step 최적화 후 정확도 저하 | 긴 cuRobo spline이 줄기 근처까지 담당 | 2-step pre-approach 구조 복원 |
| 파지 후 딸기가 분리되지 않음 | 정면 역진만으로 줄기 분리력 부족 | BASE `-Z 40mm` detach pull 추가 |
| 성공 여부를 코드가 모름 | 그리퍼 hardware read 실패 | `GRASP_UNVERIFIED`로 기록하고 육안 판정과 분리 |

### AI 활용

Claude Code와 Codex에는 긴 ROS 로그 비교, 코드 경로 탐색, IK branch 및 파라미터
가설 정리를 맡겼다. 그러나 어떤 접근 정책을 실기에 적용할지, 어느 파라미터를
유지·폐기할지, 안전하게 다음 테스트로 넘어갈지는 민석이 실제 로봇의 움직임과
과실 접촉을 관찰하여 최종 결정했다.

## 성과

### Before / After

| 항목 | Before | After |
| --- | --- | --- |
| SW 단일 딸기 수확 | 접근 실패, 측방 진입, 얕은 진입, 미분리 반복 | 민석 육안 기준 줄기 파지·분리 성공 사례 확보 |
| 최종 접근 방식 | cuRobo spline 중심, TCP 경로 불명확 | 정지 후 TOOL `+Z` 직선 진입 |
| 줄기 목표 높이 | 보정 없음/불충분 | `+30mm` Z bias |
| 추가 진입 | 부족하거나 실험별 변동 | 현재 요청/실행 `65mm` |
| 분리 동작 | 정면 역진 중심 | BASE `-Z 40mm` detach pull |
| 최신 완료 시퀀스 시간 | 기준 측정 없음 | 약 `36.4초` (`16:01:02.792`~`16:01:39.233`) |
| 자동 수확 성공률 | 측정 불가 | **측정 필요** (`GRASP_UNVERIFIED`) |
| 3D target 오차 | 측정 없음 | **측정 필요** |
| 손상률 / drop rate | 측정 없음 | **측정 필요** |

실사용 관점에서의 성과는 단순히 한 번 딸기를 딴 것이 아니라, 수확 실패를
perception, planning, execution, verification 단계로 구분하고 재현 가능한 로그를
남기는 구조를 만든 것이다. 이는 농가 환경에서 작업자 개입 원인을 줄이고,
복잡한 과실에 대해 어떤 조건에서 자동 수확을 포기해야 하는지 설명하는 기반이
된다.

## 자소서 소재 메모

### 드러난 역량

- 실패를 “로봇이 이상하다”로 끝내지 않고 단계별 원인과 로그로 분리하는 습관
- 자동화가 어려운 마지막 접촉 구간에는 결정적인 직선 동작을 배치하는 판단력
- 라이브러리 결과를 그대로 신뢰하지 않고 실제 하드웨어 관찰로 검증하는 태도
- 작은 SW 단일 과실부터 성공 조건을 만든 뒤 복잡 셀로 확장하는 단계적 접근

### 면접 답변: 어려웠던 점

> 가장 어려웠던 점은 planner 로그상 성공과 실제 수확 성공이 달랐다는 점입니다.
> cuRobo는 목표 자세까지 유효한 joint trajectory를 만들었지만, 줄기 근처에서
> TCP가 곡선으로 접근하며 잎과 과실을 밀었습니다. 저는 문제를 planner 실패로만
> 보지 않고 최종 접촉 구간의 실행 방식 문제로 정의했습니다. 이후 cuRobo는
> pre-approach와 안전성 검증에 사용하고, 마지막 구간은 로봇 native MoveLine으로
> 직선 진입하도록 분리했습니다. 또한 파지 후 BASE -Z 40mm 분리 동작과 JSONL
> 로그를 추가해 SW 단일 과실의 육안 수확 성공 사례를 확보했습니다.

### 한 줄 자소서

> 저는 AI 모션 플래너의 성공 로그를 그대로 믿지 않고, 실제 로봇의 접촉 실패를
> 계층별로 분석해 수확 가능한 hybrid motion pipeline으로 개선했습니다.

---

## 포트폴리오 한 섹션

### 단일 딸기에서 시작한 설명 가능한 수확 모션

**WHY**

저 민석은 화면에서 딸기를 검출하는 데서 끝나는 데모가 아니라, 실제 줄기를
잡아 과실을 분리하는 시스템을 만들고자 했다. 가장 쉬운 SW 단일 과실에서도
로봇은 IK branch를 크게 바꾸거나 잎을 밀었고, `pick_complete`가 출력돼도 실제
수확은 실패했다. 이 차이를 해결하지 않으면 군집·가림 환경으로 확장할 수 없다고
판단했다.

**HOW**

실패를 perception, planning, execution, verification 네 단계로 나누고 매 실행을
JSONL과 Git commit으로 연결했다. cuRobo는 긴 구간의 IK·충돌·관절 branch 검증에
적합하지만 최종 TCP 직선성을 보장하지 않는다는 점을 확인했다. 그래서
pre-approach까지는 cuRobo와 MoveSplineJoint를 사용하고, 줄기 근처에서는 정지 후
Doosan MoveLine으로 직선 진입하도록 역할을 분리했다. 실제 관찰을 바탕으로
목표 높이를 `+30mm`, 추가 진입을 `65mm`, 분리 동작을 BASE `-Z 40mm`로
조정했다. Claude Code와 Codex는 로그 비교와 가설 탐색에 활용했고, 실기 적용과
최종 파라미터 결정은 내가 직접 내렸다.

**WHAT**

SW 단일 과실에서 줄기 파지와 분리의 육안 성공 사례를 확보했고, 최신 완료
시퀀스는 약 `36.4초`가 소요됐다. 접근·파지·분리·scan pose 복귀 이벤트를
재생 가능한 JSONL로 남겼다. 다만 자동 파지 센서 판정은 아직
`GRASP_UNVERIFIED`이며, 반복 성공률·3D 오차·손상률은 측정 필요다. 다음 단계는
시간 단축과 정량 검증, marker 기반 place, NE 군집 및 NW 가림 셀 수확이다.

---

## 노션 팀 페이지 복붙용

# SW셀(단일딸기) 수확

## 목표

가장 단순한 SW 단일 과실에서 `인식 -> 접근 -> 줄기 파지 -> 분리 -> scan pose
복귀`를 먼저 검증하고, 이후 NE 군집 및 NW 가림 환경으로 확장한다.

## 시스템 파이프라인

```text
RealSense RGB-D
 -> YOLO seg + pose fusion
 -> ripe 필터 + 줄기 keypoint 안정화
 -> depth + hand-eye + FK로 base_link target 생성
 -> scan_executor가 한 target 전달
 -> cuRobo가 pre-approach/endpoint IK·충돌·branch 검증
 -> MoveSplineJoint로 pre-approach 이동
 -> 정지 후 MoveLine TOOL +Z 직선 진입
 -> gripper close
 -> BASE -Z 40mm detach pull
 -> 직선 retreat
 -> cuRobo + MoveSplineJoint로 SW scan pose 복귀
 -> runtime JSONL에 결과 기록
```

## 핵심 판단

- cuRobo 단독 joint trajectory는 줄기 근처 TCP 직선성을 보장하지 않아 hybrid
  motion으로 변경했다.
- 최종 접촉 구간은 MoveLine으로 고정하고, cuRobo는 긴 구간과 실행 가능성
  검증에 집중시켰다.
- planner 완료와 실제 수확 성공을 구분하기 위해 결과를 `GRASP_UNVERIFIED`로
  유지하고 육안 결과를 별도로 기록한다.

## 주요 트러블슈팅

| 문제 | 원인 | 해결 |
| --- | --- | --- |
| 아래에서 위로 접근 | orientation에 `+14.7deg` 상승 성분 | 수평/pitch 후보 탐색 |
| 옆으로 휘며 접근 | cuRobo joint spline의 TCP 경로 | stop-then-straight MoveLine |
| 줄기를 빗겨가거나 얕음 | target/TCP/실물 위치 오차 | Z `+30mm`, extra advance `65mm` |
| 파지 후 미분리 | 정면 후퇴만으로 분리력 부족 | BASE `-Z 40mm` detach pull |
| 성공 여부 미확정 | gripper hardware read 실패 | 자동 결과 `GRASP_UNVERIFIED`, 육안 라벨 분리 |

## 현재 결과

- SW 단일 과실 줄기 파지·분리: **육안 성공 사례 확보**
- 최신 완료 시퀀스 시간: **약 36.4초**
- pre-approach: **현재 60mm 재검증 설정**
- grasp Z bias: **+30mm**
- extra advance: **65mm**
- detach pull: **BASE -Z 40mm**
- 자동 성공률 / 3D 오차 / 손상률: **측정 필요**
- place 및 NE/NW 수확: **미완료**

## 다음 작업

1. 동일 SW 조건에서 최소 30회 반복해 성공률, cycle time, 손상률을 측정한다.
2. planning latency와 대기 시간을 분리해 전체 시간을 단축한다.
3. fresh tray localization으로 marker place와 release를 검증한다.
4. NE 군집 과실, NW 잎/줄기 가림 순서로 난이도를 높인다.
5. rule-based 접근이 실패한 장면은 향후 VLA supervisor의 reobserve/skip 판단
   데이터로 저장한다.

## 당시 감정 메모

`[민석 작성] 반복 실패 때 느낀 답답함, 첫 실제 분리 성공 순간, 로그 기반으로
원인을 좁혀가며 확신이 생긴 과정 등을 2~3문장으로 추가`
