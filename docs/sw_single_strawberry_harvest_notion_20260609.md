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
 -> TOOL -Z 직선 retreat
 -> cuRobo + MoveSplineJoint로 SW scan pose 복귀
 -> runtime JSONL에 결과 기록
```

## 왜 Hybrid Motion을 사용했나

- cuRobo는 긴 이동 구간의 IK, 충돌, 관절 branch 검증에 사용한다.
- 줄기 근처에서는 joint trajectory가 TCP 직선을 보장하지 않아 MoveLine으로
  정면 직선 진입한다.
- 파지 후 정면 후퇴만으로 딸기가 분리되지 않아 BASE `-Z 40mm` pull을 추가했다.

## 주요 트러블슈팅

| 문제 | 원인 | 해결 |
| --- | --- | --- |
| 아래에서 위로 접근 | orientation에 `+14.7deg` 상승 성분 | 수평/pitch 후보 탐색 |
| 옆으로 휘며 접근 | cuRobo joint spline의 TCP 경로 | stop-then-straight MoveLine |
| 줄기를 빗겨가거나 얕음 | target/TCP/실물 위치 오차 | Z `+30mm`, extra advance `65mm` |
| 1-step 접근 정확도 저하 | 긴 spline이 줄기 근처까지 담당 | 2-step pre-approach 복원 |
| 파지 후 미분리 | 정면 후퇴만으로 분리력 부족 | BASE `-Z 40mm` detach pull |
| 성공 여부 미확정 | gripper hardware read 실패 | `GRASP_UNVERIFIED`와 육안 판정 분리 |

## 현재 결과

- SW 단일 과실 줄기 파지·분리: **민석 육안 성공 사례 확보**
- 최신 완료 시퀀스 시간: **약 36.4초**
- pre-approach: **현재 60mm 재검증 설정**
- grasp Z bias: **+30mm**
- extra advance: **65mm**
- detach pull: **BASE -Z 40mm**
- 자동 성공률 / 3D target 오차 / 손상률: **측정 필요**
- marker place 및 NE/NW 수확: **미완료**

`grasp OK`와 `pick_complete`는 실제 수확 성공을 뜻하지 않는다. 현재 자동
판정은 `GRASP_UNVERIFIED`이므로 육안 관찰과 분리해 기록한다.

## 파지 성공 정량 지표

“그리퍼가 목표 자세에 도달했다”와 “딸기를 실제로 잡았다”를 분리해 측정한다.

| 지표 | 계산식 / 판정 기준 | 현재 상태 |
| --- | --- | --- |
| Grasp verification coverage | 유효한 gripper position 판독 횟수 / close 시도 횟수 | hardware read 실패로 측정 필요 |
| Contact detection rate | `GRASP_CONTACT_DETECTED` / 유효 판독 횟수 | position `< 665` 기준 구현됨, 검증 필요 |
| Empty grasp rate | `GRASP_EMPTY` / 유효 판독 횟수 | position `>= 665` 기준 구현됨, 검증 필요 |
| Detach success rate | 줄기에서 분리된 과실 수 / 파지 시도 수 | 현재 육안 라벨, 자동화 필요 |
| Retention success rate | retreat 후에도 그리퍼에 유지된 과실 수 / 분리 성공 수 | 측정 필요 |
| End-to-end harvest success | 파지 + 분리 + retreat 유지 성공 수 / 전체 시도 수 | 측정 필요 |
| Grasp verifier precision/recall | 자동 접촉 판정과 사람 라벨 비교 | 라벨 데이터 수집 필요 |

최소 실험 단위는 동일 SW 조건 `30회`로 설정한다. 각 시도는
`GRASP_CONTACT_DETECTED`, `GRASP_EMPTY`, `GRASP_UNVERIFIED`, `DETACH_FAIL`,
`DROP_DURING_RETREAT`, `SUCCESS` 중 하나로 기록한다.

현재 gripper position 기반 판정은 “jaw가 완전히 닫히지 않았으므로 중간에
무언가가 있다”는 간접 판정이다. 잎이나 파츠 접촉도 성공으로 오인할 수 있으므로,
최종 수확 성공률은 retreat 후 카메라 확인 또는 force/current/tactile 센서와
함께 검증해야 한다.

## AI 활용 및 의사결정

Claude Code와 Codex는 ROS 로그 비교, 코드 탐색, IK branch와 파라미터 가설
정리에 활용했다. 실기 적용 여부, 안전한 테스트 순서, 최종 모션 정책과
파라미터는 민석이 실제 로봇의 움직임을 관찰하여 결정했다.

## 다음 작업

1. 동일 SW 조건에서 최소 30회 반복해 성공률, cycle time, 손상률을 측정한다.
2. planning latency와 대기 시간을 분리해 전체 시간을 단축한다.
3. fresh tray localization으로 marker place와 release를 검증한다.
4. NE 군집 과실, NW 잎/줄기 가림 순서로 난이도를 높인다.
5. rule-based 접근 실패 장면을 향후 VLA의 reobserve/skip 판단 데이터로 저장한다.

## 당시 감정 메모

`[민석 작성] 반복 실패 때 느낀 답답함, 첫 실제 분리 성공 순간, 로그 기반으로
원인을 좁혀가며 확신이 생긴 과정을 추가`
