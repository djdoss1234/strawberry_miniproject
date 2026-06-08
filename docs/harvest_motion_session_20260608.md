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
- overview 이동은 직선 역주행 성공 후에만 허용한다.
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
4b overview after straight reverse retreat
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
6. 직선 후퇴가 완료된 뒤에만 overview로 이동하는지 확인한다.
7. 실제 줄기 파지 여부는 `PICK COMPLETE`가 아니라 영상/사람 관찰로 별도 기록한다.
