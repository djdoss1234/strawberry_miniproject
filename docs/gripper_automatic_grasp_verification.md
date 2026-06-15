# Gripper Automatic Grasp Verification

## 확인된 통신 구조

RH-P12-RN-A 공식 ROS 2 패키지는 `dynamixel_hardware_interface`를 사용하여
`Present Position`, `Present Velocity`, `Present Current`를 읽을 수 있다.

현재 수확 시스템은 그리퍼를 USB DYNAMIXEL 포트에 직접 연결하지 않고, Doosan
툴 플랜지 시리얼을 통해 Modbus RTU로 제어한다. 따라서 별도 패키지로 통신
소유권을 넘기지 않고 기존 Modbus 노드에 동일한 상태 피드백을 추가했다.

## 추가된 자동 피드백

```text
/dsr01/gripper/read_state
/dsr01/gripper/present_position
/dsr01/gripper/present_current_raw
```

`read_state` 응답 예시:

```json
{"position": 620, "current_raw": 145}
```

현재 자동 판정 순서:

```text
close 명령
 -> Present Position + Present Current 읽기
 -> position >= 665: GRASP_EMPTY
 -> position < 665: GRASP_CONTACT_DETECTED
 -> runtime JSONL에 position/current 기록
```

여기서 `GRASP_CONTACT_DETECTED`는 실제 줄기 파지 성공이 아니라 **접촉 후보**다.
잎, 넓은 꼭지, 다른 물체가 끼어도 같은 결과가 나올 수 있다.

전류 임계값은 아직 활성화하지 않는다. 빈 파지, 줄기 파지, 잎 파지의 실제
`current_raw` 분포를 수집한 뒤 다음 파라미터로 활성화한다.

```text
grasp_current_contact_threshold_raw
```

## 첫 검증 절차

새 바이너리를 사용하려면 기존 gripper service node를 재시작한다.

상태 읽기:

```bash
ros2 service call /dsr01/gripper/read_state std_srvs/srv/Trigger "{}"
```

최소 수집 조건:

| 조건 | 권장 반복 | 사람 라벨 |
| --- | ---: | --- |
| 빈 파지 | 10회 | empty |
| 실제 줄기 파지 | 10회 | stem |
| 잎 또는 넓은 꼭지 접촉 | 10회 | non-stem contact |

조건별로 물체 상태를 한 번 준비한 뒤 아래 명령을 실행하면 10회 상태 판독과
JSONL 저장은 자동으로 수행된다.

```bash
python3 scripts/collect_gripper_feedback.py --condition empty
python3 scripts/collect_gripper_feedback.py --condition stem
python3 scripts/collect_gripper_feedback.py --condition leaf_or_non_target
```

저장 위치:

```text
logs/gripper_calibration/YYYY-MM-DD/gripper_feedback.jsonl
```

사람 라벨은 제거하지 않는다. 자동 판정의 precision/recall을 계산하기 위한
정답 데이터로 사용한다. 자동 판정이 충분히 검증된 뒤 사람 입력 빈도를 줄인다.

개입을 줄이기 위해 초기 보정 표본 각 10회와 실패/무작위 표본만 라벨링하고,
실제 성공률을 보고하는 정식 반복 실험에서만 모든 시도를 라벨링한다.
