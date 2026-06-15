# SafeGrasp 단독 검증 절차

## 목적

`dsr_gripper_tcp`의 전류/위치 양방향 통신으로 빈 파지, 줄기 파지, 비목표
접촉을 어느 정도 구분할 수 있는지 먼저 검증한다. 이 단계가 끝나기 전에는
cuRobo 수확 시퀀스의 기존 close/read-state 경로를 교체하지 않는다.

`grasp_detected=true`는 접촉 또는 부하 감지이며, 목표 줄기 파지나 최종 수확
성공을 뜻하지 않는다.

## 시작 전 확인

1. 로봇과 그리퍼 주변을 비우고 비상정지 사용이 가능한 상태로 둔다.
2. `workspace_scan`과 cuRobo planner를 종료한다.
3. `bringup.launch.py`는 로봇 드라이버와 `/dsr01/drl/*` 서비스를 위해
   유지하되, 이 bringup이 실행한 기존 `/dsr01/gripper_service_node` 프로세스는
   종료한다. 두 그리퍼 제어 노드를 동시에 실행하지 않는다.
4. Doosan bringup과 그리퍼 전원/RS-485 연결을 확인한다.
5. 아래 명령의 controller host는 현재 로봇 `110.120.1.66`을 사용한다.

활성 노드 확인:

```bash
ros2 node list | grep -E 'gripper|curobo'
ros2 service list | grep '/dsr01/drl/'
```

## 확인된 TCP/DRL 방식 제한

2026-06-15 실기에서 `dsr_gripper_tcp` TCP 연결은 성공했지만, DRL 내부
`flange_serial_*` 초기화가 `status 3 (IO error)`로 실패했다. 기존 ROS
`/dsr01/gripper/flange_serial_*` 방식은 동일한 port/slave/baud 설정으로
동작하므로, 현재 로봇에서는 아래 ROS 어댑터를 우선 사용한다.

## SafeGrasp ROS 어댑터 실행

기존 `bringup.launch.py`와 `/dsr01/gripper_service_node`를 유지한다.
`workspace_scan`과 cuRobo planner만 종료한다.

```bash
source ~/doosan_ws/install/setup.bash
ros2 run e0509_gripper_description safe_grasp_ros_adapter.py
```

준비 확인:

```bash
ros2 action list | grep safe_grasp
```

현재 어댑터는 보정용 1단계 구현이다. 닫기 전과 닫기 후 상태를 각각 읽어
전류/위치 차이를 판정하며, 연속 피드백과 `object_lost` 감시는 아직 하지 않는다.

## 단일 시험

명령에는 실제 동작을 명시하는 `--execute`가 반드시 필요하다. 먼저 그리퍼
사이에 아무것도 없는 빈 파지부터 수행한다.

```bash
cd ~/doosan_ws/src/e0509_gripper_description
source ~/doosan_ws/install/setup.bash

python3 scripts/run_safe_grasp_trial.py \
  --condition empty \
  --target-position 700 \
  --max-current 400 \
  --current-delta-threshold 120 \
  --notes "empty calibration 1" \
  --execute
```

줄기 파지 시험:

```bash
python3 scripts/run_safe_grasp_trial.py \
  --condition stem \
  --target-position 700 \
  --max-current 400 \
  --current-delta-threshold 120 \
  --notes "manual stem fixture calibration 1" \
  --execute
```

잎 또는 비목표 접촉 시험:

```bash
python3 scripts/run_safe_grasp_trial.py \
  --condition leaf_or_non_target \
  --target-position 700 \
  --max-current 400 \
  --current-delta-threshold 120 \
  --notes "leaf contact calibration 1" \
  --execute
```

자동 저장 위치:

```text
logs/gripper_calibration/YYYY-MM-DD/safe_grasp_trials.jsonl
```

## 최소 검증 수량과 판정

- 빈 파지 5회
- 줄기 파지 5회
- 잎/비목표 접촉 5회

기록된 `present_current`, `current_delta`, `final_position` 분포를 비교해
`max_current`와 `current_delta_threshold`를 선정한다. 빈 파지를 접촉으로
판정하거나 잎 접촉을 줄기 파지로 판정할 수 있으므로, SafeGrasp 결과만으로
최종 수확 성공을 선언하지 않는다.

## 다음 단계

1. 보정 결과로 자동 접촉 판정 임계값을 확정한다.
2. cuRobo planner의 close/read-state 구간을 SafeGrasp action client로 교체한다.
3. SafeGrasp feedback/result를 runtime JSONL에 자동 기록한다.
4. NW 수확 실험에서 자동 판정과 사람 라벨을 비교한다.
