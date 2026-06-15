# RH-P12-RN-A 양방향 통신 진단 - 2026-06-15

## 결론

현재 Doosan E0509 툴 플랜지 RS-485 구성에서는 RH-P12-RN-A의 Modbus 쓰기
명령은 동작하지만, FC03 읽기 요청에 대한 수신 데이터가 반환되지 않는다.
따라서 현재 배선/펌웨어/설정 그대로는 전류 기반 SafeGrasp 자동 판정을
수확 시퀀스에 사용하지 않는다.

## 확인한 항목

- 통신 설정: port 1, slave ID 1, 57600 baud, 8-N-1
- 기존 ROS flange-serial 방식의 position/torque 쓰기 동작
- `dsr_gripper_tcp` DRL TCP 서버 연결 성공
- DRL 내부 INITIALIZE: `status 3 (IO error)` 반복
- ROS `/dsr01/gripper/read_state`: `position=-1`, `current_raw=-1`
- 요청 직후 0.1초 단위 반복 raw read: 항상 0바이트
- FC03 읽기 주소:
  - Torque Enable 256
  - Goal Current 275
  - Moving Status 285
  - Present Current/Position 287~291

모든 주소에서 `flange_serial_write`는 성공했지만 `flange_serial_read`는
`success=false`, `size=0`이었다.

## 의미

| 기능 | 현재 판단 |
| --- | --- |
| position/torque 쓰기 | 가능 |
| Goal Current 쓰기 | 프로토콜상 가능, 실기 검증 필요 |
| Present Current/Position 읽기 | 현재 구성에서 실패 |
| 전류 기반 실시간 접촉 판정 | 현재 불가 |
| 전류 기반 object-lost 판정 | 현재 불가 |
| 사람/영상 기반 수확 성공 판정 | 계속 사용 가능 |

`grasp_detected=false` 로그는 빈 파지를 정확히 판정한 것이 아니라, 상태 읽기
실패로 판정을 수행하지 못한 결과다.

## 다음 선택지

### A. 공식 ROBOTIS USB/DYNAMIXEL 통신 사용

그리퍼를 USB DYNAMIXEL 인터페이스에 연결하고 공식
`RH-P12-RN-A`/`dynamixel_hardware_interface`의 state interface에서
Present Current/Position을 읽는다. 양방향 전류 제어가 가장 명확한 경로지만,
현재 툴 플랜지 배선 구성을 변경해야 한다.

### B. 제공된 Doosan RH-P12-RN-DR Skill 검증

제조사 Skill의 `RH_GET_STATUS`, `RH_GET_CONFIG` 반환값을 DRL/ROS로 전달할 수
있는지 확인한다. 공식 매뉴얼상 상태 반환은 Hardware Error, Moving Status,
Moving이며 Present Current/Position 반환은 명시돼 있지 않다.

### C. 기존 수확 실험 계속 진행

기존 쓰기 기반 그리퍼 동작을 유지하고, 파지/분리/유지는 영상 및 사람 라벨로
판정한다. 자동 KPI는 planning, execution, cycle time, result code를 계속
수집한다.

## 보존 도구

```bash
python3 scripts/diagnose_gripper_read.py \
  --execute-read --start-register 287 --count 5
```

이 도구는 그리퍼를 움직이지 않고 FC03 raw read 응답만 확인한다.
