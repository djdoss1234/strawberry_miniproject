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

사람 라벨은 제거하지 않는다. 자동 판정의 precision/recall을 계산하기 위한
정답 데이터로 사용한다. 자동 판정이 충분히 검증된 뒤 사람 입력 빈도를 줄인다.
