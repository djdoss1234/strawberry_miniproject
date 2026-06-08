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

## 단순 base Z 상승 실패와 줄기 방향 보정

`GRASP_Z_BIAS=+15mm` 적용 실행에서는 로그상 목표가 실제로 상승했다.

```text
raw=(-351,672,451)mm
grasp=(-351,672,466)mm
z_bias=+15mm
```

하지만 물리적으로는 이전과 비슷하게 과실을 스쳤다. 원인은 “줄기 위쪽”을
`base_link +Z`로만 해석한 것이다. 이번 target의 실제 줄기는 KP0에서 KP2로 갈 때
X/Y/Z가 모두 변하는 대각선 방향이었다. 따라서 Z만 올리면 높이는 바뀌어도 그리퍼
중심선이 줄기에서 옆으로 벗어날 수 있다.

수정:

```text
old target = KP0 + [0, 0, 15mm]
new target = KP0 + normalize(KP2 - KP0) * min(10mm, 0.8 * |KP2-KP0|)
planner GRASP_Z_BIAS = 0
```

이제 fusion node가 전체 줄기 방향을 따라 KP0에서 최대 10mm 이동한 점을 publish하며,
planner는 별도 base Z 보정을 중복 적용하지 않는다. JSONL의 target quality에
`grasp_target_source`, `grasp_offset_from_kp0_m`, `grasp_direction_base`,
`grasp_target_base_m`을 저장한다.

## 15:44~15:49 재시험: 높이 변화가 작아 보인 원인과 무동작 원인 분리

두 번의 재시험에서 관찰된 현상은 서로 다른 원인이었다.

### 첫 시도: 새 줄기 방향 target은 적용됐지만 물리 높이 차이가 작음

`curobo_planner_node_20260608T154424-15c612e0.jsonl`에서는 새 fusion target이
planner에 전달되어 실제 접근까지 실행됐다.

```text
input target: (-345.9, 683.9, 469.9)mm
prepared target after wall clamp: (-345.9, 672.0, 469.9)mm
planner extra base-Z bias: 0mm
```

이전 `base_link +Z 15mm` 실행 목표가 약 `466mm`였기 때문에, 새 줄기 방향 목표의
실제 Z는 이전 실행 위치보다 약 `4mm`만 높았다. 따라서 코드 변경은 적용됐지만
실기 영상에서는 이전과 거의 같은 높이로 보일 수 있다. 다음 높이 튜닝은
`KP0`, `KP2`, 최종 target의 실제 차이를 JSONL로 비교한 뒤 별도 보정량으로
조정해야 한다.

### 두 번째 시도: 잘못된 fusion guard 때문에 target이 발행되지 않음

15:47 및 15:49 실행에서 planner는 정상적으로 Ready 상태였지만
`pick_sequence_start`가 없었고, scan executor는 `SCANNED_EMPTY`로 종료했다.
fusion 로그에는 다음 reject가 반복됐다.

```text
stem_side_keypoint_not_inside_matched_ripe_mask
```

이 조건은 과실 몸체 segmentation mask 안에 줄기 쪽 `KP0` 또는 `KP1`이 있어야
한다고 가정했다. 그러나 현재 모델에서 segmentation mask는 과실 몸체이고,
KP0/KP1은 줄기 쪽 점이므로 마스크 밖에 있는 것이 정상이다. 특히 유효한
`center` 매칭까지 이 조건이 차단하여 검출이 보여도 로봇이 움직이지 않는
간헐적 `SCANNED_EMPTY`를 만들었다.

수정 후에는 이 잘못된 hard reject를 제거했다. 대신 기존의 다음 검사는 유지한다.

- seg class가 ripe인지와 HSV ripe metrics
- 필요한 stem keypoint confidence
- KP0/KP1/KP2의 유효 depth 및 3D 줄기 형상
- 여러 프레임 동안의 stable target tracking

따라서 다음 단일 SW 검증의 우선 확인 항목은 scan pose 도착 후 12초 안에
`pick_sequence_start`가 생성되는지와, JSONL의 `grasp_target_base_m`이
실제 원하는 줄기 중심 높이에 대응하는지이다.

## 현재 줄기 방향 target에서 물리 높이 10mm 추가

KP0에서 KP2 방향으로 최대 10mm 이동하는 보정은 줄기 방향을 따르므로,
줄기가 대각선이면 실제 `base_link Z` 상승량은 10mm보다 작다. 15:44 실행에서도
새 target은 이전 실행 target보다 약 4mm만 높아 실기에서 차이가 거의 보이지 않았다.

현재 줄기 방향 target을 유지하면서 물리적으로 확실하게 10mm 더 위를 겨냥하도록
fusion target 생성에 독립적인 파라미터를 추가했다.

```text
stem target = KP0 + normalize(KP2-KP0) * stem_grasp_offset
final target = stem target + [0, 0, grasp_target_base_z_trim_m]

grasp_target_base_z_trim_m = +0.010m
planner GRASP_Z_BIAS = 0.000m
```

즉, 줄기 방향 보정 뒤에 `base_link +Z 10mm`를 한 번만 적용한다. planner 쪽
`GRASP_Z_BIAS`는 0으로 유지하여 중복 상승을 방지한다. fusion 시작 로그와 JSONL
target quality의 `grasp_target_base_z_trim_m`으로 실제 적용 여부를 확인할 수 있다.

다음 실기 검증은 단일 SW target, 저속, 명확히 보이는 줄기에서 수행하며,
그리퍼 파츠가 과실/잎을 스치는지와 줄기가 파츠 중앙에 들어오는지를 확인한다.

## SW 반복 수확용 marker 기반 place 연결

2026-06-04에 이동한 계란판에서도 ArUco 기반 15개 slot grid가 다시 계산되는 것을
확인했다. 당시 검증된 계란판 관측 자세는 다음과 같다.

```text
TRAY_VIEW_JOINTS_DEG = [-0.02, -2.41, 111.87, 175.94, -31.34, 93.42]
```

최신 `~/Downloads/share_tray/output/tray_cells_*.json`을 place 좌표 source로 읽는
marker place 단계를 현재 수확 planner의 직선 retreat 뒤에 연결했다.

```text
SW scan / target lock
 -> grasp
 -> TOOL -Z straight reverse retreat
 -> overview transfer
 -> tray-view
 -> marker-derived slot above (+100mm)
 -> marker-derived 60mm standoff release position
 -> gripper release
 -> slot above retreat
 -> SW pick-start scan pose
 -> 다음 target
```

현재 marker output의 `position_tcp_mm`는 그리퍼 파츠 끝이 tray plane보다 60mm
위에 위치하는 좌표다. 첫 release baseline은 이 검증된 standoff를 그대로 사용하며,
실제 slot 내부 깊은 삽입은 아직 수행하지 않는다.

안전 파라미터:

```text
enable_marker_place_sequence:=false
execute_marker_place_release:=false
marker_place_max_age_sec:=300.0
marker_place_above_clearance_m:=0.100
tray_cells_json:=""  # 빈 값이면 최신 tray_cells_*.json 사용
```

첫 검증은 release를 끈 preview mode로 수행한다.

```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p execute_marker_place_release:=false
```

이 모드에서는 수확·retreat 후 slot above까지만 이동하고 정지한다.
`pick_complete`도 발행하지 않으므로 scan executor가 다음 동작으로 넘어가지 않는다.
above clearance와 실제 계란판 위치를 확인한 뒤에만 release를 승인한다.

```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p execute_marker_place_release:=true
```

### 16:50 / 16:55 반복 시도: 실제 파지·분리 검증 부재와 stale tray 차단

두 시도 모두 commit `fc8a437`에서 marker place preview를 켠 상태로 수행했다.

| run_id | 사용자 물리 관찰 | 로그에서 확인되는 사실 | 최종 정지 원인 |
| --- | --- | --- | --- |
| `20260608T165005-ba59d38c` | 잎에 밀려 실제 파지 실패 | target 접근, gripper close, 직선 reverse retreat 완료 | tray localization age `974s`가 허용 `300s` 초과 |
| `20260608T165503-d7ae2a59` | 그리퍼가 잡았으나 딸기가 줄기에서 분리되지 않음 | target 접근, gripper close, 직선 reverse retreat 완료 | tray localization age `1255s`가 허용 `300s` 초과 |

이번 결과로 다음을 확인했다.

- 현재 로그의 `grasp OK`는 실제 파지 또는 분리 성공이 아니라 **grasp 목표 자세 도달**
  성공이다.
- 그리퍼 명령과 retreat가 성공해도 잎 접촉, 빈 파지, 미분리 상태를 자동 판정하지
  못한다.
- 두 시도에서 place로 가지 않은 직접 원인은 실제 파지 실패 인지가 아니라
  `MARKER_PLACE_BLOCKED: tray localization stale` 안전 검사였다.
- stale/preview/place 실패 뒤 persistent sequence hold latch가 정상 동작하여,
  planner restart 전까지 후속 pick target을 차단했다.
- runtime JSONL은 `logs/` ignore 정책으로 로컬 실험 자산으로 보존되며 git에는
  올리지 않는다.

다음 최우선 구현은 retreat 후 `VERIFY_GRASP / VERIFY_DETACH` 단계다. 실제 성공
근거가 없으면 place를 시작하지 않고 `GRASP_EMPTY`, `DETACH_FAIL`,
`GRASP_UNVERIFIED` 중 하나로 종료해야 한다. 또한 로그 문구 `grasp OK`는 실제
성공으로 오해되지 않도록 `GRASP_POSE_REACHED`로 변경한다.

marker place를 다시 검증하려면 수확 직전에 tray localization을 갱신해야 한다.
실제 release는 fresh tray JSON, 물리적 분리 확인, slot above clearance 확인을
모두 만족한 단일 시도에서만 승인한다.

다음 조건에서는 place와 자동 scan continuation을 차단하고 현재 자세에서 정지한다.

- marker localization JSON 없음
- marker 결과가 300초보다 오래됨
- slot 목표가 guarded workspace 밖
- overview/tray-view cuRobo transfer 실패
- above/release/above-retreat MoveLine 실패
- place 후 SW scan pose 복귀 실패

`PLACE_SEQUENCE_COMPLETE_UNVERIFIED`는 경로와 release 명령이 끝났다는 뜻이며,
딸기가 실제 slot 안에 놓였다는 성공 판정은 아니다.

### 16:37 marker place preview 실기 관찰

- 사용자가 물리적으로 딸기 분리를 확인한 뒤 marker place preview를 수행했다.
- 최신 tray localization 파일을 정상적으로 읽었으며, slot0 above 목표는
  `base_link xyz=[471.6, -311.5, 726.3]mm`로 계산되었다.
- 로봇은 overview와 tray-view를 경유해 slot0 above에 도달했다.
- 계란판 마커는 eye-in-hand 영상에서 그리퍼 사이로 보였으므로, 이번 정지는
  마커 가림이나 좌표 미검출 때문이 아니다.
- 실행 파라미터가 `execute_marker_place_release:=false`였기 때문에
  `MARKER_PLACE_PREVIEW_HOLD` 상태에서 의도적으로 정지했다.
- `Fusion Detection` 화면은 딸기 검출 overlay이며 tray 15-slot 좌표를 표시하는
  화면이 아니다. tray 좌표 로딩 여부는 planner 로그와 tray localization 결과로
  확인한다.

preview hold 이후에도 fusion node가 새 pick target을 계속 발행하여, planner가
tray pose에서 새로운 수확 계획을 시작할 수 있는 문제가 확인되었다. 이를 막기
위해 다음 상태에서 planner restart 전까지 새 pick target을 거부하는 persistent
sequence hold latch를 추가했다.

- marker place preview hold
- marker place 실패
- straight reverse retreat 실패
- pick-start scan pose 복귀 실패

실제 release 단계는 slot above의 물리 clearance를 확인한 뒤 아래 파라미터로만
승인한다.

```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p enable_marker_place_sequence:=true \
  -p execute_marker_place_release:=true
```

## VERIFY_GRASP / VERIFY_DETACH 구현 — 2026-06-08 저녁

### 동기

두 번의 실기에서 모션 시퀀스는 완료되었으나 실제 파지 여부를 확인할 수 없었다.

```text
run ba59d38c: 잎에 밀려 실제 파지 실패, 로그에는 grasp OK 기록
run d7ae2a59: 줄기를 잡았지만 딸기가 분리되지 않음, 로그에는 grasp OK 기록
```

두 경우 모두 `MARKER_PLACE_FAILED`(tray 좌표 만료)로 pick이 중단되었지만,
`grasp OK`가 실제 파지 성공이 아닌 자세 도달 이벤트임이 명확히 구분되지 않았다.

### 구현 내용 (commit 3a35fa3)

**C++: `gripper_service_node.cpp`**

기존 `read_present_position()` 메서드를 ROS Trigger 서비스로 노출한다.

```text
서비스: /dsr01/gripper/read_position
응답:   success=true, message=str(position)
       position=-1 이면 가상 모드 또는 serial 오류
```

**Python: `curobo_planner_node.py`**

```text
GRASP_EMPTY_POSITION_THRESHOLD = 665
GRASP_VERIFY_TIMEOUT_SEC       = 5.0
```

파지 분류 기준:

| result_code | 조건 | 의미 |
|---|---|---|
| GRASP_EMPTY | pos >= 665 | jaw 완전 닫힘, 아무것도 없음 |
| GRASP_CONTACT_DETECTED | 0 <= pos < 665 | jaw 중간 정지, 줄기 접촉 추정 |
| GRASP_UNVERIFIED | pos = -1 또는 서비스 미응답 | 가상 모드 또는 판독 실패 |

pick 시퀀스 변경점:

```text
grasp OK 로그 → GRASP_POSE_REACHED
grasp_approach_complete JSONL 이벤트 → grasp_pose_reached
gripper close (2.0s)
 → VERIFY_GRASP (read_position 서비스 호출)
   → verify_grasp JSONL 기록
retreat (MoveLine TOOL -Z)
 → VERIFY_DETACH (상태 기록; 센서 없음 → DETACH_UNVERIFIED)
   → verify_detach JSONL 기록
Place 게이트:
  GRASP_CONTACT_DETECTED → place 허용
  GRASP_EMPTY / GRASP_UNVERIFIED → place_gate_blocked JSONL, place 건너뜀
```

retreat는 grasp_result와 무관하게 항상 실행된다. 벽 앞에 멈춰 있으면 안 되기 때문이다.

### 검증 상태

```text
Python syntax check:  passed
colcon build:         passed (e0509_gripper_description)
git diff --check:     clean
Physical test:        pending
```

### 다음 실기 확인 항목

1. close 직후 `VERIFY_GRASP` 로그와 `present_pos=` 값 확인
2. 실제 줄기 파지 시 `GRASP_CONTACT_DETECTED` 나오는지 확인
3. 빈 파지 시 `GRASP_EMPTY (pos=700)` 나오는지 확인
4. `GRASP_EMPTY`에서 place 건너뛰고 scan 복귀하는지 확인
5. 가상 모드(real_mode=false) → `GRASP_UNVERIFIED` 나오는지 확인
6. `GRASP_EMPTY_POSITION_THRESHOLD=665` 적절성 — 실제 줄기 파지 시 pos값 측정 후 조정 필요

### 임계값 조정 안내

RH-P12-RN-A에서 줄기를 잡을 때 실제 위치(present_position)를 로그로 확인한 뒤,
`GRASP_EMPTY_POSITION_THRESHOLD`를 조정한다.

```text
예: 줄기 파지 시 pos=620 관찰 → threshold를 650 정도로 낮출 수 있음
    빈 파지 시 pos=700 → threshold 665 이상이면 GRASP_EMPTY 정상 판정
```
