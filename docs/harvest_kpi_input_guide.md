# 수확 실험 KPI 입력 가이드

## 언제 입력하는가

개발 중 모든 수확 시도를 입력할 필요는 없다. 초기 자동 파지 판정 보정용 표본,
실패 시도, 무작위 표본에 아래 명령을 실행한다. 최종 성공률을 보고하는 정식
반복 실험에서는 모든 시도를 입력한다.

```bash
cd ~/doosan_ws/src/e0509_gripper_description
python3 scripts/label_harvest_attempt.py
```

도구는 가장 최근 `curobo_planner_node` runtime JSONL의 마지막 수확 시도를
자동으로 선택한다. 다른 run을 판정할 때만 `--runtime <파일>`을 지정한다.

## 사람이 확인하여 입력할 항목

| 입력 시점 | 사람이 확인할 내용 | 입력 항목 |
| --- | --- | --- |
| 그리퍼 close 직후 | 실제 목표 딸기의 **줄기**를 잡았는가 | 실제 줄기 파지 |
| detach pull 직후 | 딸기가 줄기/고정부에서 분리됐는가 | 분리 성공 |
| retreat 완료 직후 | 딸기를 놓치지 않고 유지했는가 | 후퇴 유지 |
| 진입 및 후퇴 전체 | 잎, 다른 딸기, 구조물에 닿았는가 | 비목표 접촉 |
| 시도 전체 | 정지, 복구, 위치 조정 등 사람이 개입했는가 | 사람 개입 |
| Place 수행 후 | 목표 slot에 정상 배치됐는가 | Place 결과 |
| 전체 자동화 검증 시 | scan 시작부터 다음 작업 준비까지 걸린 시간 | 전체 작업 시간(초) |

전체 작업 시간은 스톱워치로 측정한 경우에만 입력하고, 모르면 Enter로 넘긴다.
자동 Pick 시퀀스 시간과 motion/planning 결과는 runtime JSONL에서 가져온다.

라벨은 실행 로그를 수정하지 않고 다음 경로에 별도로 누적된다.

```text
logs/human_labels/YYYY-MM-DD/harvest_attempt_labels.jsonl
```

## KPI 확인

```bash
python3 scripts/summarize_harvest_kpis.py
```

runtime JSONL만으로 자동 계산 가능한 계획/시간/접촉 후보 KPI:

```bash
python3 scripts/summarize_runtime_kpis.py --cell root/nw
```

핵심 KPI는 다음 6개다.

1. 실제 줄기 파지 성공률
2. 최종 Pick 성공률: 줄기 파지, 분리, 후퇴 유지가 모두 성공
3. 평균 Pick 시퀀스 시간
4. Place 성공률
5. 전체 작업 시간
6. 사람 개입률

## Place 안전 게이트

기본값에서는 `GRASP_CONTACT_DETECTED`일 때만 Place를 허용한다.
`GRASP_UNVERIFIED` 상태에서 Place를 시험해야 한다면 사람이 실제 파지를 확인한
단일 저속 테스트에서만 다음 파라미터를 명시한다.

```text
-p allow_unverified_grasp_place:=true
```

이 옵션은 자동 파지 검증을 대신하지 않는다. 사용한 모든 시도에 사람 판정
라벨을 반드시 남긴다.
