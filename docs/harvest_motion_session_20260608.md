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
Physical robot validation: pending
```

## 다음 실기 확인 항목

1. SW 단일 딸기에서 접근과 close를 저속으로 확인한다.
2. close 직후 로그에 아래 항목이 출력되는지 확인한다.

   ```text
   RETREAT_STRAIGHT_REVERSE TOOL -Z 130.0mm
   ```

3. 그리퍼 방향과 관절 branch가 유지된 채 진입 경로를 그대로 빠져나오는지 확인한다.
4. 직선 후퇴가 완료된 뒤에만 overview로 이동하는지 확인한다.
5. 실제 줄기 파지 여부는 `PICK COMPLETE`가 아니라 영상/사람 관찰로 별도 기록한다.
