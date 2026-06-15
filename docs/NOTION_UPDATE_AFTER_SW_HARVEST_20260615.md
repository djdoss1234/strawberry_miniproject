# SW 단일딸기 수확 이후 고도화 진행 상황

> 이 문서는 기존 노션 페이지 `SW셀(단일딸기) 수확 모션 구현 및 실기 검증`
> 이후 추가된 작업을 정리한 후속 페이지다.

## 1. 이번 단계 목표

SW 단일딸기 수확 성공 사례를 만든 뒤, 단일 동작 성공에서 끝나지 않고 다음
단계로 확장했다.

```text
SW 수확 모션 안정화
 -> 실제 TCP/파지점 정합 개선
 -> 꺾인 줄기 대응 파지 모션 개선
 -> Pick 이후 계란판 Place 연결
 -> 15개 slot 격자 생성 및 안전 검증
 -> KPI 자동 수집 구조 구축
 -> 전류 기반 파지 성공 자동 판정 검증
 -> NW 잎/줄기 가림 셀로 확장 준비
```

---

## 2. SW 수확 모션 추가 개선

### 2.1 실측 TCP 기준 적용

기존에는 flange에서 원래 그리퍼까지의 길이와 연장 파츠 길이가 혼재되어
실제 파지 중심과 planner의 TCP가 어긋날 가능성이 있었다.

실측 결과:

| 항목 | 실측값 |
| --- | --- |
| flange → 파츠 끝단 | 약 `270mm` |
| 실제 파지 중심 | 끝단보다 약 `10mm` 뒤 |
| planner 기준 TCP | flange에서 약 `260mm` 파지 중심 |

실측 TCP 모델에서는 cuRobo의 ee_link 자체를 실제 파지 중심으로 사용하고,
과거의 중복 길이 보정과 기본 extra advance를 제거했다.

**[사진 1 삽입 위치: 이 표 아래]**

- 필요 자료: flange, 원래 그리퍼, 연장 파츠 끝단, 실제 파지 중심을 자로 측정한 사진
- 권장 캡션: `실측 파지 중심을 기준으로 planner TCP를 재정의하여 중복 길이 보정을 제거했다.`

### 2.2 꺾인 줄기 target 보정

고정 높이와 수평 접근만 사용하면 줄기가 꺾인 경우 실제 줄기 옆을 지나가는
문제가 있었다.

이를 줄기 keypoint의 국소 방향과 midpoint를 사용하도록 수정했다.

```text
KP0/KP1 국소 줄기 방향 계산
 -> 줄기 주변의 목표 구간 선택
 -> 수평 pre-approach
 -> 열린 그리퍼로 줄기 방향 하강
 -> KP1 부근에서 close
```

**[사진 2 삽입 위치: 위 시퀀스 아래]**

- 필요 자료: YOLO 화면에서 KP0/KP1/KP2와 실제 목표점을 표시한 캡처
- 권장 캡션: `단일 고정점 대신 줄기 keypoint의 국소 방향을 이용해 꺾인 줄기의 목표점을 보정했다.`

### 2.3 파지 모션 변경

기존:

```text
줄기 목표 높이로 바로 수평 진입
 -> close
 -> detach pull
```

현재:

```text
position 600 상태로 줄기 위쪽에 수평 진입
 -> 열린 상태로 BASE -Z 30mm 하강
 -> KP1 부근에서 position 700 close
 -> BASE -Z 40mm detach pull
 -> TOOL -Z retreat
```

이 방식은 연장 파츠가 잎이나 딸기 몸체를 먼저 밀어내는 현상을 줄이기 위한
변경이다.

**[영상 1 삽입 위치: 현재 시퀀스 아래]**

- 필요 자료: 수평 진입 후 열린 상태 하강, close, detach pull, retreat가 연속으로 보이는 영상
- 권장 캡션: `그리퍼를 연 상태로 줄기를 따라 내려온 뒤 KP1 부근에서 닫고, 아래 방향으로 분리한다.`

---

## 3. 수확 시퀀스 시간 및 planning 최적화

기존 SW 기준 단일 run의 Pick 시간은 약 `36.4초`였다. 이후 실제 병목을
다음처럼 분리했다.

| 병목 | 개선 내용 |
| --- | --- |
| 불필요한 후보 탐색 | target 주변 국소 줄기 midpoint와 후보 순서 조정 |
| 긴 settle/wait | pre-approach 및 파지 대기시간 축소 |
| 저속 운전 | place spline 전 operation speed `100%` 설정 |
| 고정 MoveLine timeout | 거리/속도/운전 속도율 기반 timeout으로 변경 |
| 긴 경로 계산 | 유효 후보를 먼저 시도하고 위험 branch는 실행 전 거부 |

구체적으로 검증된 수평 `0°` orientation을 첫 후보로 이동하고, 초기 IK seed를
`48 -> 24`, close 안정화 대기를 `1.5초 -> 0.3초`, 파지 상태 확인 timeout을
`20초 -> 5초`로 줄였다. 실패 시에는 나머지 orientation 후보로 fallback한다.

단, 개선 후 전체 Pick/Place 시간은 아직 동일 조건 반복 실험으로 확정하지
않았다. 다음 NW/SW 반복 실험부터 runtime JSONL로 평균과 표준편차를 측정한다.

**[그래프 1 삽입 위치: 위 표 아래]**

- 필요 자료: 개선 전후 `planning latency`, `Pick sequence time`, `wait/execute time` 분해 그래프
- 현재 상태: 개선 후 반복 데이터 수집 필요
- 권장 캡션: `단일 cycle의 총시간뿐 아니라 계획, 대기, 실행 시간을 분리하여 병목을 추적한다.`

---

## 4. 계란판 Place 연결 및 검증

### 4.1 Slot 기준과 격자 생성

계란판은 다음 순서로 정의했다.

```text
Slot0  Slot3  Slot6  Slot9  Slot12
Slot1  Slot4  Slot7  Slot10 Slot13
Slot2  Slot5  Slot8  Slot11 Slot14
```

실측 티칭 좌표:

| 기준 slot | 역할 |
| --- | --- |
| Slot0 | 기준 release pose |
| Slot1 | row pitch 계산 |
| Slot3 | column pitch 계산 |

Slot0/1/3 티칭값에서 row/column pitch를 계산해 15개 slot 목표를 생성했다.
현재 방식은 **marker localization이 아닌 고정 tray 격자 baseline**이다.

초기에는 marker localization 결과로 tray가 이동해도 slot을 자동 생성하려고
했지만, 실기에서 localization timestamp stale, tray-view orientation,
도달 불가능한 top-down pose 문제가 이어졌다. 따라서 marker 방식은 폐기한
것이 아니라 후속 비교 대상으로 남기고, 먼저 티칭 격자로 Pick→Place 실행
구조와 안전 게이트를 검증했다.

**[사진 3 삽입 위치: 슬롯 도식 아래]**

- 필요 자료: 계란판 위에 Slot0~14 번호를 표시한 top-view 이미지
- 권장 캡션: `Slot0/1/3 티칭값을 기준으로 3×5 계란판 격자를 생성했다.`

### 4.2 Place 검증 결과

| 구분 | 상태 | 방식 |
| --- | --- | --- |
| Slot0, Slot1, Slot3, Slot4 | 실기 성공 | Above 이동 후 BASE `-Z` 하강 |
| Slot2 | 부분 성공 | 30도 tilt, 약 3cm 오차 |
| Slot5 | 안전 차단 | cuRobo 경로가 수직선에서 `100.8mm` 이탈 |
| 나머지 slot | 미검증 | 후속 검증 예정 |

Slot5는 경로 생성 자체는 성공했지만 계란판 위 수직 하강선에서 크게 벗어나
release 전에 차단했다.

```text
ROW2_DESCENT_LINE_CHECK max_deviation=100.8mm limit=20.0mm
TAUGHT_TRAY_SLOT5_PLACE_BLOCKED
```

이는 place 실패이면서 동시에 위험 경로를 실행하지 않은 safety guard 성공이다.
row2는 Cartesian constraint/waypoint IK 또는 collision geometry 보강 전까지
강제 실행하지 않는다.

**[영상 2 삽입 위치: Place 결과표 아래]**

- 필요 자료: Slot0 또는 Slot4 정상 Place 영상
- 권장 캡션: `티칭 기준 격자로 생성한 row0/1 slot에 Pick 이후 자동 Place를 수행했다.`

**[그래프 2 삽입 위치: Slot5 설명 아래]**

- 필요 자료: Above→release 목표 수직선과 실제 계획 경로, 최대 편차 `100.8mm` 표시
- 권장 캡션: `계획 성공 여부와 별개로 Cartesian line deviation을 검사해 위험한 Place 경로를 차단했다.`

---

## 5. 파지 성공 자동 판정 - SafeGrasp

기존에는 `grasp pose 도달`, `close 명령 성공`, `실제 수확 성공`을 자동으로
구분하지 못했다. 특히 기존 `/dsr01/gripper/read_state` 경로는 `-1/-1`을
반환하여 결과가 `GRASP_UNVERIFIED`로 남았다.

2026-06-15 원본
`Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge` 패키지의
`/gripper_service/safe_grasp` 액션을 실기 검증했다.

확인 결과:

| 항목 | 결과 |
| --- | --- |
| TCP bridge 연결 | 성공 |
| position/current 실시간 feedback | 성공 |
| Goal Current 제한 | `400` 설정 확인 |
| 빈 파지 자동 판정 | 성공 |
| 빈 파지 최종값 | position `700`, current `8` |
| 결과 메시지 | `target reached without grasp` |

SafeGrasp가 자동으로 알 수 있는 것은 **접촉 후보 또는 빈 파지**다. 잎을 잡아도
접촉으로 판단할 수 있으므로 실제 줄기 파지, 분리, 유지 여부는 초기에는
영상 라벨과 비교해야 한다.

**[그래프 3 삽입 위치: 결과표 아래]**

- 필요 자료: 빈 파지 SafeGrasp의 시간별 position/current 선 그래프
- 데이터: `logs/gripper_calibration/2026-06-15/safe_grasp_trials.jsonl`
- 권장 캡션: `빈 파지에서 position은 700까지 닫혔지만 current가 낮아 grasp_detected=false로 자동 판정됐다.`

### 아직 하지 않은 것

- 줄기/잎 조건별 SafeGrasp 반복 보정
- current/current_delta 임계값 확정
- cuRobo pick 시퀀스에 SafeGrasp action 연결
- SafeGrasp 기반 최종 수확 성공률 측정

---

## 6. KPI 자동 수집 및 사람 입력

### 자동 기록되는 항목

| 영역 | 자동 KPI |
| --- | --- |
| Planning | plan success/fail/reject, planning latency |
| Execution | MoveSplineJoint/MoveLine 결과, Pick 시퀀스 시간 |
| Gripper | position, current, current delta, 접촉 후보, 빈 파지, object lost |
| Safety/Recovery | spline jump, line deviation, hold/recovery 원인 |

### 사람 또는 영상 라벨이 필요한 항목

| 항목 | 이유 |
| --- | --- |
| 실제 줄기 파지 | SafeGrasp는 잎 접촉과 줄기 접촉을 구분하지 못함 |
| 분리 성공 | detach 동작 실행과 실제 분리는 다름 |
| 후퇴 유지 | retreat 후 딸기를 계속 들고 있는지 확인 필요 |
| 비목표 접촉 | 잎/다른 과실/구조물 접촉 확인 |
| Place 성공 | 목표 slot 착지와 유지 확인 |

자동 KPI 도구:

```bash
python3 scripts/summarize_runtime_kpis.py --cell root/nw
python3 scripts/generate_harvest_kpi_report.py --cell root/nw
```

수동 라벨:

```text
reports/harvest_kpi/manual_labels_root_nw.csv
```

**[그래프 4 삽입 위치: KPI 표 아래]**

- 필요 자료: NW 반복 실험 후 생성한 KPI dashboard
- 구성 권장: 최종 Pick 성공률, 평균 cycle time, plan/reject 분포,
  SafeGrasp 판정 분포, 사람 개입률
- 현재 상태: 도구 구현 완료, NW 반복 데이터 수집 전

---

## 7. 다음 단계 - NW 잎/줄기 가림 셀

Place row2는 위험 경로 차단까지 검증하고 잠시 중단했다. 다음 우선순위는
`root/nw` 잎/줄기 가림 환경이다.

```text
SafeGrasp 줄기/잎/빈 파지 보정
 -> cuRobo pick 시퀀스에 SafeGrasp 연결
 -> NW 중앙 view에서 target/KP1 가시성 확인
 -> 보이는 줄기만 기존 KP1 rule로 Pick
 -> 가려진 target은 강제 진입 대신 reobserve/skip
 -> multi-view 관측 적용
 -> AnyGrasp/GraspGen offline 후보와 기존 rule 비교
```

AnyGrasp/GraspGen은 일반 6-DoF grasp 후보 생성기이므로 바로 실제 로봇에
연결하지 않는다. 가림으로 줄기 point cloud가 없으면 먼저 multi-view
재관측이 필요하며, 후보는 줄기 근접도, 접근 방향, IK, 충돌, branch 필터를
통과한 경우에만 평가한다.

**[사진 4 삽입 위치: 다음 단계 시퀀스 아래]**

- 필요 자료: NW 셀에서 잎/줄기가 가려진 딸기 정면 사진
- 권장 캡션: `다음 단계에서는 가림 조건에서 재관측과 접근 가능 판단을 검증한다.`

---

## 8. 현재 성과와 제한

### 확인된 성과

- SW 단일딸기의 수평 접근, KP1 파지, 아래 방향 분리, retreat 성공 사례 확보
- 실측 TCP와 국소 줄기 방향을 반영한 target/파지 모션 개선
- Pick 이후 Slot0/1/3/4 자동 Place 성공
- 위험한 row2 Place 경로를 Cartesian 편차로 실행 전 차단
- 실행 로그와 KPI 보고서 자동화 기반 구축
- 원본 SafeGrasp 패키지로 position/current 및 빈 파지 자동 판정 확인

### 아직 주장하면 안 되는 것

- 반복 실험 기반 높은 수확 성공률
- 모든 15개 slot 자동 Place 성공
- 이동한 tray에 대한 marker 기반 자동 Place 완성
- NW/NE 복잡 셀 수확 성공
- SafeGrasp만으로 실제 줄기 파지 및 최종 수확 성공 판정
- AnyGrasp/GraspGen 적용 완료

## 9. 관련 파일

```text
scripts/curobo_planner_node.py
scripts/strawberry_fusion_node.py
scripts/run_safe_grasp_trial.py
scripts/set_experiment_context.py
scripts/summarize_runtime_kpis.py
scripts/generate_harvest_kpi_report.py
docs/HANDOFF_20260614_PLACE_TRAY_GRID.md
docs/HANDOFF_20260615_SAFEGRASP_NW_NEXT.md
docs/SAFE_GRASP_STANDALONE_TEST_20260615.md
docs/NW_OCCLUSION_KPI_AND_GRASP_DIRECTION_20260615.md
docs/HARVEST_EXPERIMENT_OPERATION_PLAN_20260615.md
```

## 10. 시각자료 준비 체크리스트

| 우선순위 | 자료 | 넣을 위치 | 현재 상태 |
| --- | --- | --- | --- |
| 필수 | SW 최신 Pick 전체 영상 | 2.3 파지 모션 변경 | 새 촬영 권장 |
| 필수 | Slot0/4 정상 Place 영상 | 4.2 Place 검증 결과 | 기존 영상 확인 필요 |
| 필수 | Slot0~14 번호가 표시된 계란판 top-view | 4.1 Slot 기준과 격자 생성 | 새 이미지 제작 필요 |
| 필수 | SafeGrasp 빈 파지 position/current 그래프 | 5. 파지 성공 자동 판정 | JSONL 확보, 그래프 생성 필요 |
| 필수 | NW 잎/줄기 가림 정면 사진 | 7. 다음 단계 | 실험 시작 전 촬영 |
| 권장 | 실측 TCP 길이 사진 | 2.1 실측 TCP 기준 적용 | 기존 실측 사진 확인 필요 |
| 권장 | KP0/KP1/KP2 및 목표점 캡처 | 2.2 꺾인 줄기 target 보정 | Fusion 화면 캡처 필요 |
| 권장 | Slot5 목표 직선과 계획 경로 비교 그래프 | 4.2 Place 검증 결과 | FK 로그 기반 생성 필요 |
| 후속 | NW KPI dashboard | 6. KPI 자동 수집 | 반복 실험 후 자동 생성 |

노션에는 모든 로그 화면을 그대로 넣지 않고, 핵심 로그 2~3줄과 이를 해석한
그래프/영상 캡션을 함께 사용한다.
