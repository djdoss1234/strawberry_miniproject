# Gripper Automatic Grasp Verification

## 확인된 통신 구조

RH-P12-RN-A 공식 ROS 2 패키지는 `dynamixel_hardware_interface`를 사용하여
`Present Position`, `Present Velocity`, `Present Current`를 읽을 수 있다.

현재 수확 시스템은 그리퍼를 USB DYNAMIXEL 포트에 직접 연결하지 않고, Doosan
툴 플랜지 시리얼을 통해 Modbus RTU로 제어한다.

2026-06-15 실기 진단에서 쓰기 명령은 동작했지만 모든 FC03 읽기 요청의 응답이
0바이트였다. 따라서 아래 자동 피드백 인터페이스는 구현돼 있지만, 현재 장비
구성에서는 유효 상태를 반환하지 못한다.

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

계획한 자동 판정 순서:

```text
close 명령
 -> Present Position + Present Current 읽기
 -> position >= 665: GRASP_EMPTY
 -> position < 665: GRASP_CONTACT_DETECTED
 -> runtime JSONL에 position/current 기록
```

여기서 `GRASP_CONTACT_DETECTED`는 실제 줄기 파지 성공이 아니라 **접촉 후보**다.
잎, 넓은 꼭지, 다른 물체가 끼어도 같은 결과가 나올 수 있다.

전류 임계값은 활성화하지 않는다. 먼저 USB/DYNAMIXEL 직접 연결 또는 제조사
Skill 연동으로 실제 상태 readback을 확보해야 한다.

```text
grasp_current_contact_threshold_raw
```

## 현재 진단 결과

상세 증거와 다음 선택지는
`docs/GRIPPER_BIDIRECTIONAL_DIAGNOSIS_20260615.md`에 기록한다. 상태 readback을
확보하기 전까지 파지/분리/유지는 영상과 사람 라벨로 판정한다.
