# 수확 접근 모션 개선 기록 - 2026-06-07

## 1. 세션 목표

`root/sw`의 단일 딸기를 대상으로, 그리퍼가 줄기를 향해 안전하고 재현 가능하게
접근하도록 수확 모션을 개선했다.

이번 세션에서 해결하려던 핵심 문제는 다음과 같다.

1. 그리퍼가 딸기 아래에서 위쪽으로 기울어진 채 접근했다.
2. 파지 위치가 확정되기 전에 팔과 손목이 계속 움직이며 진입했다.
3. cuRobo joint-space spline만으로는 마지막 TCP 진입이 정면 직선인지 보장하기 어려웠다.
4. 로그의 `grasp OK` 및 `PICK COMPLETE`가 실제 줄기 파지 성공처럼 보였지만,
   실제로는 모션/시퀀스 완료 이벤트였다.

## 2. 시작 상태

기존 기본 자세:

```python
WALL_QUAT_WXYZ = [0.548415, -0.439294, 0.424628, 0.570923]
```

이 자세에서 gripper local `+Z` 접근 방향은 다음과 같다.

```text
approach_dir = [-0.0359, 0.9667, 0.2534]
elevation = +14.7 deg
```

즉 TCP는 주로 벽 방향인 `+Y`로 향하지만 `+Z` 성분도 커서, 실제 그리퍼가
딸기 아래에서 위로 올라가는 대각선 접근을 수행했다.

## 3. 실패와 원인 분석

### 3.1 완전 수평 쿼터니언 강제 실패

시험값:

```python
WALL_QUAT_WXYZ = [0.488, -0.506, 0.494, 0.512]
```

계산상 접근 방향은 거의 수평이었다.

```text
approach_dir = [-0.0360, 0.9994, 0.0002]
elevation = +0.01 deg
```

그러나 SW scan pose에서 해당 자세의 안전한 IK branch를 찾지 못해 grasp motion이
실행되지 않았다. 로봇은 기존의 위로 기울어진 scan pose에 남았고 파지도 수행하지
않았다.

결론:

- 수학적으로 올바른 수평 쿼터니언이라고 해서 현재 관절 구성에서 실행 가능한 것은 아니다.
- grasp motion이 실행되지 않았을 때 보이는 자세는 요청한 grasp 자세가 아니라 기존 scan pose다.
- 완전 수평값 하나를 강제하는 방식은 폐기하고, 현재 SW branch를 유지하는 후보 자세를 단계적으로 탐색해야 한다.

### 3.2 기존 pitch retry 회전 순서 오류

기존 코드는 다음 순서로 쿼터니언을 보정했다.

```python
q_retry = WALL_QUAT * q_delta
```

이 방식은 gripper local-frame 축을 회전한다. 주석에는 위아래 pitch 보정이라고
적혀 있었지만, local X축으로 `-25 deg`를 적용해도 elevation이 약
`14.7 deg -> 13.4 deg`로만 줄었다.

수정:

```python
q_retry = q_delta * WALL_QUAT
```

base-frame X축에서 pre-multiply하여 실제 접근축 elevation을 변경했다.

허용 후보:

| Base X 보정 | 실제 접근 elevation |
| ---: | ---: |
| `-14.7 deg` | 약 `0.0 deg` |
| `-10.0 deg` | 약 `+4.7 deg` |
| `-5.0 deg` | 약 `+9.7 deg` |

기존 `+14.7 deg` 자세는 fallback에서 제거했다. 세 후보가 모두 실패하면 위로
기울어진 자세로 억지 실행하지 않고 파지를 중단한다.

### 3.3 연속 spline 진입 문제

초기 2단계 접근은 다음 두 cuRobo plan을 연속 실행했다.

```text
scan pose -> pre-approach -> grasp
```

pre-approach와 grasp가 별도 plan이더라도 실행 사이 정지 시간이 거의 없고,
두 번째 실행 역시 joint-space `MoveSplineJoint`이므로 TCP 정면 직선 진입을
보장하지 못했다.

## 4. 구현한 최종 접근 구조

현재 구현:

```text
1. cuRobo가 pre-approach와 grasp endpoint를 모두 미리 계획
2. IK, collision, operational joint range, branch/swing 안전 검사
3. scan pose -> pre-approach: cuRobo + MoveSplineJoint
4. pre-approach 도착 후 1.0초 완전 정지
5. pre-approach -> grasp: Doosan MoveLine, TOOL +Z 상대 직선 이동
6. 도착 후 0.5초 정지
7. gripper close
8. 성공한 동일 orientation을 유지해 retreat 계획
```

현재 파라미터:

```python
PRE_APPROACH_OFFSET = 0.18
PRE_APPROACH_SETTLE_SEC = 1.0
FINAL_APPROACH_VEL_MM_S = 20.0
FINAL_APPROACH_ACC_MM_S2 = 30.0
FINAL_APPROACH_SETTLE_SEC = 0.5
GRASP_RETRY_OFFSETS = [0.040, 0.050, 0.065, 0.080]
```

최종 직선 진입 거리:

```python
final_approach_distance = PRE_APPROACH_OFFSET - used_grasp_offset
```

`used_grasp_offset=0.040m`이면 `140mm`를 TOOL `+Z` 방향으로 직선 이동한다.

## 5. 2026-06-07 SW 실기 결과

확인한 로그:

```text
allowed grasp pitch corrections/elevations:
  -14.7 -> -0.0deg
  -10.0 -> +4.7deg
  -5.0 -> +9.7deg

PRE_APPROACH_REACHED
FINAL_APPROACH_STRAIGHT TOOL +Z 140.0mm vel=20.0mm/s
grasp OK
  offset=+0.040m
  variant=('base', [1, 0, 0], -14.7)
  approach_dir=[-0.0359, 0.9994, -0.0002]
  elevation=-0.0deg
3 close gripper
```

물리 관찰:

- 그리퍼 접근 방향은 의도한 정면 방향으로 수정되었다.
- pre-approach에서 멈춘 뒤 정면으로 천천히 진입하는 동작이 확인되었다.
- 그러나 최종 진입 깊이가 부족해 줄기를 실제로 파지하지 못했다.

판정:

```text
Orientation correction: experimentally observed working
Stop-then-straight approach: experimentally observed working
Actual stem grasp: failed / not achieved
```

주의:

- 로그의 `grasp OK`는 최종 접근 모션 완료를 의미한다.
- `3 close gripper`는 close service 호출을 의미한다.
- `PICK COMPLETE`는 시퀀스 종료 이벤트다.
- 위 이벤트들은 실제 줄기 파지 성공 증거가 아니다.

## 6. 현재 남은 문제

### P0. 최종 진입 깊이 부족

명령상 TOOL `+Z`로 `140mm`를 이동했지만 물리 파츠가 줄기를 충분히 감싸지 못했다.

가능한 원인:

1. `GRASP_OFFSET=0.040m`가 실제 파츠/줄기 위치에 비해 너무 보수적이다.
2. perception target인 KP0/줄기 좌표가 실제 파지점보다 앞쪽에 있다.
3. `GRIPPER_LEN=0.160m`와 실제 15.8cm 연장 파츠/TCP 기준 사이에 오차가 있다.
4. Doosan TOOL frame 원점과 cuRobo `gripper_rh_p12_rn_base` 기준이 정확히 일치하지 않는다.
5. gripper approach position `600` 상태에서 턱 형상상 추가 진입 여유가 필요하다.

### P0. 실제 파지 성공 검증 부재

현재 stroke/close command만으로 실제 줄기 접촉 및 파지 성공을 판정할 수 없다.
사람 관찰 또는 영상 라벨이 필요하며, 이후 motor current/position feedback 또는
post-grasp vision 검증을 추가해야 한다.

## 7. 다음 세션 작업 순서

1. SW 단일 딸기만 사용하고 주변을 비운 저속 검증을 유지한다.
2. 현재 `140mm` 직선 진입 후 그리퍼 끝과 목표 줄기 사이 잔여 거리를 실측한다.
3. 벽/딸기 몸통/주변 줄기 여유를 확인한 뒤 추가 진입량을 `5~10mm` 단위로 적용한다.
4. 추가 진입량을 별도 파라미터로 만든다.

   ```python
   FINAL_APPROACH_EXTRA_M = 0.0
   distance = PRE_APPROACH_OFFSET - used_grasp_offset + FINAL_APPROACH_EXTRA_M
   ```

5. `+10mm`, 필요 시 `+20mm` 순으로 검증한다. 한 번에 큰 값을 적용하지 않는다.
6. 실제 파지 성공을 사람 라벨과 영상으로 기록한다.
7. SW에서 성공 후 NE cluster, NW occlusion 순으로 확장한다.

## 8. 안전 조건

- 새 직선 진입량 시험 전 비상정지 접근성을 확보한다.
- place sequence는 table collision 위험 때문에 계속 비활성 상태로 유지한다.
- J1 반대 branch, J4/J6 spline jump, operational limit, trajectory swing 검사를 유지한다.
- 모든 수평 후보가 실패하면 기존 위 기울기 자세로 fallback하지 않는다.
- 진입 깊이를 늘릴 때 wall clearance와 15.8cm 연장 파츠 collision model을 함께 확인한다.

## 9. 관련 파일

- `scripts/curobo_planner_node.py`
- `config/curobo/e0509_gripper.yml`
- `config/curobo/e0509_spheres.yml`
- `docs/runs/RUN-20260607-001_sw_horizontal_straight_approach.log`
- `docs/system_architecture.md`
- `docs/project_retrospective_portfolio_roadmap.md`
