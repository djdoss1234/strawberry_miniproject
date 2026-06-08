# SW 수확 Retreat 수정 기록 - 2026-06-08

## 관찰 결과

SW 단일 딸기 접근 및 그리퍼 close까지 실행되었으나, 파지 직후 retreat 계획이
거부되어 의도한 후퇴 동작을 수행하지 못했다.

주요 로그:

```text
FINAL_APPROACH_STRAIGHT TOOL +Z 130.0mm
3 close gripper
4 retreat (CuRobo)
Cartesian plan rejected: J1 swing 75.7deg > 75.0deg
Retreat plan failed - overview 직행
```

## 원인

최종 접근은 확정된 자세로 TOOL `+Z` 직선 이동을 사용했지만, retreat는 접근
경로를 역주행하지 않고 별도의 Cartesian 목표를 다시 생성했다.

```text
grasp pose -> 36cm 후퇴 + 5cm 상승 Cartesian 목표
```

cuRobo가 이 새 목표를 풀면서 J1이 크게 회전하는 IK branch를 선택했고, 실제
trajectory의 J1 swing이 안전 기준 75.0도를 0.7도 초과하여 실행 전에 거부되었다.

안전 필터가 위험한 경로를 올바르게 막은 사례이며, 기준을 단순히 완화하는 방식은
사용하지 않았다.

## 수정 내용

파지 직후에는 새 retreat IK를 계산하지 않고, 실제로 진입했던 거리를 동일 자세로
직선 역주행한다.

```text
pre-approach
 -> TOOL +Z 130mm 직선 진입
 -> gripper close
 -> TOOL -Z 130mm 직선 역주행
 -> 안전 거리 확보 후 overview 계획
```

추가 안전 동작:

- 직선 역주행이 실패하면 overview 직행을 금지한다.
- 실패 시 현재 자세와 gripper close 상태를 유지한다.
- pick 시작 scan pose 복귀는 직선 역주행 성공 후에만 허용한다.
- J1/J4/J6 operational limit 및 swing 검사는 유지한다.

## 검증 상태

```text
Code change: implemented
Python syntax check: passed
Straight reverse retreat physical validation: passed
```

## 실기 결과 추가

첫 번째 SW 딸기는 실제로 파지되었고, 새 직선 역주행 retreat도 정상 동작했다.

```text
FINAL_APPROACH_STRAIGHT TOOL +Z
3 close gripper
RETREAT_STRAIGHT_REVERSE TOOL -Z
4b return to pick-start scan pose after straight reverse retreat
```

남은 딸기 시도에서는 retreat까지 정상 완료했지만, 목표 줄기보다 옆으로 접근하여
파지하지 못했다.

첫 성공 시도와 두 번째 실패 시도의 planner 입력 목표 차이:

```text
first target:  raw=(-143,672,538)mm
second target: raw=(-118,672,573)mm
difference:    X=+25mm, Z=+35mm
```

두 번째 접근의 140mm 직선 진입에서 자세 기울기로 발생하는 X 변화는 약 5mm다.
따라서 옆 접근의 주원인은 retreat 또는 MoveLine이 아니라 fusion node가 발행한
KP0 줄기 목표 좌표의 프레임 간 흔들림으로 판단했다.

### Fusion 목표 안정화 수정

기존에는 4회 검출 후 최근 측정값 비중이 큰 EMA 좌표를 바로 발행했다. 한두 프레임의
KP0 오검출이 로봇 목표에 크게 반영될 수 있었다.

수정 후에는 다음 조건을 모두 만족한 목표만 발행한다.

- 최근 9개 KP0 3D 위치의 좌표별 중앙값을 목표로 사용
- 최소 7개 위치 샘플 필요
- 중앙값 기준 최대 공간 분산 12mm 이하
- 화면 HUD와 ROS 로그에 sample 수와 spread를 표시

```text
Published stable pick target xyz=(...)m samples=9 spread=...mm
```

이 변경은 target 흔들림을 줄이는 것이며, 실제 줄기 keypoint 자체가 일관되게 잘못
검출되는 경우까지 해결하지는 않는다.

```text
Fusion median/spread stabilization: implemented and built
Fusion stabilization physical validation: pending
```

## 다음 실기 확인 항목

1. 남은 SW 딸기에서 fusion HUD의 `s=...mm`와 publish 로그를 확인한다.
2. `spread <= 12mm`인 목표만 발행되는지 확인한다.
3. 접근 전 발행된 목표 좌표와 실제 줄기 위치가 일치하는지 화면으로 확인한다.
4. close 직후 로그에 아래 항목이 출력되는지 확인한다.

   ```text
   RETREAT_STRAIGHT_REVERSE TOOL -Z 130.0mm
   ```

5. 그리퍼 방향과 관절 branch가 유지된 채 진입 경로를 그대로 빠져나오는지 확인한다.
6. 직선 후퇴가 완료된 뒤에만 pick 시작 scan pose로 복귀하는지 확인한다.
7. 실제 줄기 파지 여부는 `PICK COMPLETE`가 아니라 영상/사람 관찰로 별도 기록한다.

## 같은 셀 연속 수확 시작 자세 수정

SW 첫 딸기 수확 후 남은 딸기를 시도했을 때, planner 시작 자세가 SW scan pose가
아니라 overview 자세였다.

```text
target=(-349,672,463)mm
start_J=[88, -94, 130, 176, -31, 93]deg
Cartesian plan rejected: J2 swing 96.3~102.8deg > 90.0deg
```

이는 목표를 빗겨간 것이 아니라 모든 후보가 실행 전에 거부되어 로봇이 움직이지 않은
상황이다. 원인은 planner가 첫 pick 후 overview로 복귀한 뒤, scan executor가 같은
SW 셀의 다음 target을 즉시 전달한 구조적 불일치였다.

수정 후:

```text
각 pick 시작 시 현재 cell scan joints 저장
 -> 직선 접근 / close / 직선 역주행
 -> 저장한 pick-start scan pose로 복귀
 -> pick_complete
 -> scan executor가 같은 셀의 다음 target 전달
```

셀 간 이동 및 전체 순회 종료 후 overview 복귀는 scan executor가 담당한다.

## SW 추가 시도: 모션 성공, 실제 줄기 파지 실패

런타임 로그:

```text
logs/runtime/2026-06-08/curobo_planner_node_20260608T115517-e65c9fb0.jsonl
```

관찰 결과:

- SW scan pose에서 시작했다.
- cuRobo pre-approach 계획, TOOL `+Z` 140mm 직선 진입, close,
  TOOL `-Z` 140mm 직선 역주행, SW scan pose 복귀가 모두 완료됐다.
- 실제로는 줄기를 제대로 잡지 못했다.
- 따라서 이번 결과는 motion sequence 완료이지만 실제 수확 성공은 아니다.

```text
motion_result: SUCCESS
harvest_result: GRASP_EMPTY / STEM_TARGET_MISS
PICK COMPLETE: sequence completion only
```

planner 입력 target은 다음과 같았다.

```text
fusion target before wall clamp: approximately (-150, 703, 570)mm
planner target after wall clamp:  (-150, 672, 575)mm
```

잎 가림 또는 pose keypoint 오검출이 유력하지만, 기존 JSONL에는 keypoint confidence,
pixel 위치, 3D keypoint geometry가 없어 원인을 확정할 수 없었다. 모션을 더 깊게
보내는 방식은 잘못된 목표로 충돌할 위험만 키우므로 적용하지 않는다.

### 줄기 목표 품질 guard 추가

fusion node가 좌표 안정성만 확인하던 구조에서, 실제 줄기 목표 신뢰도도 함께
검사하도록 변경했다.

- KP0/KP1 중 하나가 매칭된 ripe 과실 mask 내부에 있어야 한다.
- 자동 pick에는 KP0/KP1/KP2가 모두 `confidence >= 0.60`이어야 한다.
- 세 keypoint 모두 유효 depth를 가져야 한다.
- KP0-KP1, KP1-KP2 길이는 각각 5~100mm 범위여야 한다.
- KP0-KP2 전체 길이는 160mm 이하여야 한다.
- KP1 fallback 파지를 제거하고 실제 파지 목표는 항상 KP0를 사용한다.
- 불확실한 target은 접근하지 않고 `pick_target_rejected` JSONL event로 남긴다.
- 발행 target에는 confidence, pixel, 3D keypoint, mask match evidence, HSV,
  줄기 segment 길이를 함께 기록한다.

이 guard는 가려진 줄기를 억지로 따는 기능이 아니다. 현재 rule-based pipeline이
신뢰할 수 없는 줄기를 거부하고, 향후 재관측/VLA 대상으로 넘길 수 있게 만드는
안전 장치다.

```text
Stem quality guard implementation: complete
Physical validation after guard: pending
```

## Stem quality guard 적용 후 로봇이 움직이지 않은 원인

품질 guard 적용 후 첫 재실행에서는 planner가 Ready 상태였지만 pick motion이
시작되지 않았다.

실행 시각 비교:

```text
AT_SCAN_POSE root/sw:             12:15:37.003
SCANNED_EMPTY / SCAN_COMPLETE:    12:15:38.506
first stable_pick_target publish: 12:15:38.681
```

fusion은 정상 target을 발행했지만 scan executor가 약 0.18초 먼저 detection window를
닫아 버렸다. 따라서 planner가 target을 받지 못한 원인은 quality guard의 전면 거부가
아니라, 짧은 scan dwell과 target 안정화 시간 사이의 race condition이었다.

수정:

- 고정 `_SCAN_DWELL_SEC`를 ROS parameter `scan_dwell_sec`로 변경
- launch 기본값을 5.0초로 설정
- fusion의 7~9 sample 안정화와 품질 검증이 완료될 시간을 확보

```text
scan_dwell_sec default: 5.0s
recommended SW test override: scan_dwell_sec:=5.0
```

## SW 잔여 과실 시도: 잎 접촉으로 파지 실패

실행 로그:

```text
logs/runtime/2026-06-08/curobo_planner_node_20260608T123424-15a5acb2.jsonl
logs/runtime/2026-06-08/strawberry_fusion_node_20260608T123424-cd862288.jsonl
```

관찰:

- target은 실행 전부터 일관되게 약 `(-347, 685, 458)mm`로 발행됐다.
- scan executor가 다른 target으로 갑자기 전환한 것은 아니다.
- planner는 첫 6개 endpoint 후보가 IK 실패한 뒤 7번째 후보에서 성공했다.
- planning에 약 17초가 걸린 뒤 115mm 직선 진입을 수행했다.
- 15.8cm 그리퍼 연장 파츠가 진입 중 잎에 닿아 잎과 과실을 함께 밀었고,
  최종 줄기 파지에 실패했다.

현재 collision world는 whiteboard와 검출된 과실 중심 sphere만 포함한다. 잎은
segmentation class나 collision geometry로 등록되지 않으므로 cuRobo는 잎 접촉을
예측하거나 회피할 수 없다.

```text
planner result: valid for current modeled world
physical harvest result: OCCLUDED_REOBSERVE_REQUIRED / GRASP_EMPTY
missing scene element: leaf geometry
```

### 중복 pre-approach 계획 제거

기존 후보 탐색 순서는 offset마다 동일한 pre-approach를 다시 계획했다.

```text
old: offset 4개 × orientation 3개마다 pre-approach 재계획
new: orientation별 pre-approach 1회 계획 후 offset endpoint만 순차 검증
```

이번 로그와 같은 첫 orientation의 세 번째 offset 성공 사례에서는 동일한
pre-approach 계획 횟수가 3회에서 1회로 줄어든다. 물리 접근 속도를 높인 것이 아니라
실행 전 중복 GPU planning latency를 제거한 변경이다.

잎 접촉 문제는 이 최적화로 해결되지 않는다. 다음 안전 개선은 RGB-D 접근 corridor
occupancy 검사 또는 leaf segmentation을 통해 잎/미지 물체를 scene에 반영하는 것이다.

## 고정 5초 dwell에서도 target 전달 실패

15:09 실행에서 planner는 Ready 상태였지만 SW scan pose에서 움직이지 않았다.

```text
AT_SCAN_POSE root/sw:             15:10:04.251
SCANNED_EMPTY / SCAN_COMPLETE:    15:10:09.256
first stable_pick_target publish: 15:10:10.840
```

첫 유효 target이 scan pose 도착 약 6.59초 후 발행되어 고정 5초 dwell을 넘겼다.
planner에는 `/dsr01/curobo/pick_pose`가 전달되지 않았고, JSONL에도
`pick_sequence_start`가 없었다.

고정 dwell을 반복해서 늘리는 대신 adaptive detection wait로 변경했다.

```text
scan pose 도착
 -> detection buffer reset
 -> 최대 12초 동안 첫 stable target 대기
 -> target 발견 즉시 다음 pick 단계 진행
 -> 12초 동안 없으면 SCANNED_EMPTY
```

이 구조는 target이 빨리 보이는 셀에서는 불필요한 dwell을 줄이고, 품질 guard와
median stabilization 때문에 target 발행이 늦어지는 경우에는 race를 방지한다.

## SW 줄기 진입 높이 10mm 추가 상승

SW 실기 시도에서 그리퍼 개도 `600`은 줄기만 파츠 사이로 들어오면 적절했지만,
파지 목표가 조금 낮아 연장 파츠가 꼭지의 넓은 부분이나 잎을 먼저 밀었다. 이 접촉으로
딸기와 줄기가 함께 움직여 최종 파지 위치가 빗나갔다.

따라서 KP0 기준 높이 보정을 기존 `+5mm`에서 `+15mm`로 변경했다. 이는 기존 실행
위치보다 실제 파지 목표를 `10mm` 위로 이동한 것이다.

```text
GRASP_Z_BIAS: +0.005m -> +0.015m
목적: 꼭지/잎 접촉 감소, 줄기 단독 진입 확률 증가
```

런타임 콘솔과 JSONL `pick_target_prepared` 이벤트에는 각각 `z_bias=+15mm`,
`grasp_z_bias_m=0.015`가 기록된다.

다음 실기 검증에서는 줄기가 파츠 사이에 들어오는지, 꼭지/잎을 먼저 미는지,
파지 후 과실이 이동하는지를 영상과 함께 수동 라벨링한다. 이 변경은 잎 geometry를
collision world에 추가한 것이 아니므로 가려진 줄기에 대한 강제 접근 해결책은 아니다.
