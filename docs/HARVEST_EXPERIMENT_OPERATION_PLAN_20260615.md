# 딸기 수확 실험 전체 운영안 - 2026-06-15

## 1. 현재 어디까지 왔는가

### 완료 또는 실기 관찰 완료

- SW 단일 딸기: 검출, 접근, 줄기 파지, 아래 방향 분리, 후퇴까지 개별 동작 성공
- 계란판 Place: Slot0, Slot1, Slot3, Slot4 검증
- 파지 모션: 수평 진입 후 그리퍼를 연 상태로 줄기를 따라 내려와 KP1 부근에서
  닫고, 아래로 분리 후 직선 후퇴
- 실측 TCP: flange에서 실제 파지 중심까지 약 `260mm` 기준 반영
- runtime JSONL: target, 계획, 실행, 파지 판정, Place 및 실패 원인 기록
- Slot5 row2: 경로가 수직선에서 `100.8mm` 이탈하여 실행 전 안전 차단

### 아직 완료되지 않은 것

- NW 잎/줄기 가림 환경 수확
- NE 군집 딸기 수확
- 그리퍼 position/current 양방향 판독 안정화 및 자동 파지 판정 검증
- row2 Place의 직선 수직 하강
- 전체 scan → pick → place → 다음 target 연속 자동화
- 충분한 반복 실험 기반 성공률

## 2. 앞으로 진행 순서

### 단계 A - 그리퍼 자동 판정 안정화

목표는 사람이 매번 파지 여부를 적지 않아도 그리퍼가 **접촉 후보**와
**빈 파지**를 자동 구분하는 것이다.

1. `/dsr01/gripper/read_state`가 유효 `position/current_raw`를 반환하는지 확인
2. 빈 파지, 줄기 파지, 잎 접촉 조건별 10회 데이터 수집
3. position/current 분포를 비교하여 임계값 설정
4. 사람/영상 정답과 자동 판정을 비교하여 precision/recall 측정

주의: 자동 판정은 줄기를 잡았는지까지 바로 알 수 없다. 잎을 잡아도 접촉으로
판정될 수 있으므로 초기에는 정답 라벨 검증이 필요하다.

### 단계 B - NW 잎/줄기 가림 수확

1. `root/nw` 실험 조건을 한 번 등록
2. NW 중앙 scan pose에서 target과 줄기 keypoint 가시성 확인
3. 줄기가 보이는 target만 기존 파지 모션으로 시도
4. 줄기가 가려진 target은 강제 진입하지 않고 reobserve/skip으로 기록
5. multi-view 관측으로 줄기 발견률 비교
6. AnyGrasp/GraspGen 후보는 offline point cloud에서 기존 KP1 방식과 비교

### 단계 C - NE 군집 및 Place 재개

- NW에서 KPI 수집 구조와 실패 분류가 안정된 뒤 NE 군집으로 확장
- 다른 딸기/잎과의 비목표 접촉률 측정
- row0/1 Place 반복 검증
- row2는 Cartesian constraint 또는 collision geometry 보강 후 재개

### 단계 D - 전체 자동화와 정식 반복 실험

- scan → target 선택 → pick → verify → place → 다음 target 연속 실행
- SW/NW/NE 조건별 최소 30회 반복
- 실제 성공률, 사이클 시간, 사람 개입률을 비교

## 3. 파지 성공은 어떻게 측정하는가

파지 성공은 하나의 신호가 아니라 단계별로 구분한다.

```text
그리퍼 상태 판독 성공
 -> 접촉 후보 감지
 -> 실제 줄기 파지
 -> 딸기 분리
 -> 후퇴 후 유지
 -> Place 성공
```

| 단계 | 판정 방법 | 자동화 상태 |
| --- | --- | --- |
| 상태 판독 | `read_state` position/current 유효 여부 | 자동, 통신 안정화 필요 |
| 접촉 후보 | position/current 임계값 | 자동 골격 구현, 보정 필요 |
| 실제 줄기 파지 | 영상/사람 정답과 접촉 위치 비교 | 초기 수동 라벨 필요 |
| 분리 성공 | 분리 후 영상 또는 향후 비전 판정 | 현재 사람/영상 라벨 |
| 후퇴 유지 | retreat 종료 후 딸기 유지 여부 | 현재 사람/영상 라벨 |
| Place 성공 | 목표 slot 착지 및 유지 | 현재 사람/영상 라벨 |

### 최종 Pick 성공 정의

```text
실제 줄기 파지 = 성공
AND 분리 = 성공
AND 후퇴 유지 = 성공
```

그리퍼가 무언가에 닿았다는 `GRASP_CONTACT_DETECTED`만으로 최종 성공을 선언하지
않는다.

## 4. KPI와 측정 방식

### 자동 측정 KPI

| KPI | 계산 방법 | 데이터 |
| --- | --- | --- |
| 후보 계획 통과율 | plan success / success+fail+reject | runtime JSONL |
| 계획 지연시간 | 각 cuRobo 계획 계산 시간 평균/분포 | runtime JSONL |
| 계획 실패/안전 거부 수 | IK fail, spline jump, line deviation 등 | runtime JSONL |
| 자동 파지 판정 가능률 | 유효 position/current 판독 / close 시도 | runtime JSONL |
| 접촉 후보/빈 파지 감지율 | 자동 결과별 비율 | runtime JSONL |
| Pick 시퀀스 시간 | target 수신부터 종료 이벤트까지 | runtime JSONL |
| 종료/복구 원인 | complete, stopped, hold 사유 | runtime JSONL |

### 정답 라벨이 필요한 KPI

| KPI | 계산 방법 | 입력 빈도 |
| --- | --- | --- |
| 실제 줄기 파지 성공률 | 줄기 파지 성공 / 전체 시도 | 정식 실험은 전부 |
| 최종 Pick 성공률 | 파지+분리+유지 성공 / 전체 시도 | 정식 실험은 전부 |
| 비목표 접촉률 | 잎/다른 과실/구조물 접촉 / 전체 시도 | 실패·표본 및 정식 실험 |
| Place 성공률 | 목표 slot 배치 성공 / Place 시도 | Place 정식 실험 |
| 사람 개입률 | 정지/복구/수동 조정 시도 / 전체 시도 | 정식 실험 |
| 자동 판정 Precision/Recall | 자동 접촉 판정과 실제 줄기 파지 정답 비교 | 초기 보정 및 정식 실험 |

개발 중에는 모든 시도를 사람이 기록하지 않는다. 자동 판정 보정을 위한 초기
표본, 실패 run, 무작위 표본만 라벨링한다. 논문/포트폴리오용 정식 반복 실험에서
모든 시도를 라벨링한다.

## 5. 실제 실행 절차

### 5-1. NW 실험 조건 등록 - 장면 변경 시 한 번

```bash
cd ~/doosan_ws/src/e0509_gripper_description
python3 scripts/set_experiment_context.py \
  --cell root/nw \
  --scene-id nw_leaf_stem_occlusion_v1 \
  --occlusion leaf_and_stem \
  --stem-shape mixed
```

### 5-2. 그리퍼 양방향 통신 확인

```bash
ros2 service call /dsr01/gripper/read_state std_srvs/srv/Trigger "{}"
```

`position=-1` 또는 `current_raw=-1`이면 수확 성공 자동 판정 실험을 시작하지 않고
통신부터 해결한다.

### 5-3. 조건별 보정 데이터 자동 수집

각 조건을 실제로 만들어 놓고 명령을 한 번 실행한다.

```bash
python3 scripts/collect_gripper_feedback.py --condition empty
python3 scripts/collect_gripper_feedback.py --condition stem
python3 scripts/collect_gripper_feedback.py --condition leaf_or_non_target
```

### 5-4. NW 단일 셀 실행

기존 launch에서 target cell만 `root/nw`로 지정한다. 초기 실험에서는 Place를
비활성화하고 Pick과 자동 판정부터 검증한다.

### 5-5. 사람 라벨이 필요한 시도만 입력

```bash
python3 scripts/label_harvest_attempt.py
```

### 5-6. 자동 KPI 확인 및 그래프 생성

터미널 요약:

```bash
python3 scripts/summarize_runtime_kpis.py --cell root/nw
python3 scripts/summarize_harvest_kpis.py
```

PNG + JSON + Markdown 보고서:

```bash
python3 scripts/generate_harvest_kpi_report.py --cell root/nw
```

출력:

```text
reports/harvest_kpi/
  kpi_dashboard_root_nw.png
  kpi_summary_root_nw.json
  kpi_report_root_nw.md
```

그래프 구성:

1. cuRobo 후보 계획 통과/실패/안전 거부 수
2. planning latency 분포
3. 접촉 후보/빈 파지/판정 불가 분포
4. 실제 줄기 파지/최종 Pick/Place/사람 개입률 및 자동 판정 Precision/Recall

## 6. AnyGrasp와 GraspGen은 언제 사용하는가

AnyGrasp/GraspGen은 일반 point cloud에서 6-DoF 파지 후보를 생성한다. 줄기 전용
알고리즘이 아니므로 바로 실제 로봇의 파지 명령으로 사용하지 않는다.

평가 순서:

1. NW RGB-D point cloud 저장
2. AnyGrasp 후보 생성
3. 줄기 ROI에서 멀거나 과실/잎 표면을 잡는 후보 제거
4. 접근 방향, IK, collision, joint branch 필터 적용
5. 기존 KP1 rule 방식과 계획 성공률, 비목표 접촉률, 최종 Pick 성공률 비교

가림으로 줄기 point cloud가 없는 경우에는 grasp generator보다 multi-view
재관측이 먼저다.

## 7. 다음 즉시 할 일

1. `read_state` 실기 응답 확인 및 `-1/-1` 문제 해결
2. 빈 파지/줄기/잎 접촉 각 10회 자동 표본 수집
3. NW 첫 장면을 등록하고 Place 없이 5회 탐색·Pick 예비 실험
4. 자동 KPI 대시보드 확인
5. 실패 원인이 가림인지, 줄기 검출 오차인지, motion 계획인지 분리
6. 이후 NW 정식 30회 반복과 AnyGrasp offline baseline 진행
