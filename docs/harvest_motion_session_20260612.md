# Harvest Motion Session - 2026-06-12

## Slot0 place reference candidate

자동 marker-place orientation sampling은 IK에 성공하더라도 J4가 크게 회전하고
계란판 위가 아닌 공중 자세를 선택했다. 이를 해결하기 위해 사용자가 계란판
slot0 근처에서 직접 확인한 place 전용 기준 자세를 기록한다.

현재 이 자세는 `above` 또는 `release`로 확정하지 않은 **수동 티칭 기준 후보**다.
검증 전에는 자동 release 목표로 사용하지 않는다.

```yaml
name: slot0_reference_candidate_20260612
frame: base
joints_deg: [5.34, 3.05, 124.87, 179.46, -3.46, 93.51]
posx_mm_deg: [441.65, 41.20, 233.76, 5.30, 131.37, -87.05]
classification: unverified_place_reference
```

### 기존 실패와의 차이

- 이전 자동 preview는 tray-view 자세에서 임의 orientation 후보를 탐색했다.
- IK 성공만으로 후보를 선택해 J4 급회전과 부적절한 공중 자세가 발생했다.
- 새 후보는 실제 계란판 근처에서 사람이 확인한 관절 브랜치이므로, 향후
  marker slot 목표를 계획할 때 preferred/reference joint pose로 사용할 수 있다.

### 다음 검증

1. 빈 그리퍼, 저속, release 비활성 상태에서 이 자세로 이동 가능 여부를 확인한다.
2. 파츠 끝단의 slot0 중심 정렬과 계란판 clearance를 육안 확인한다.
3. 이 자세가 `above`인지 `release`인지 분류한다.
4. 안전한 `above`와 `release`를 각각 확보한 뒤에만 자동 place에 연결한다.
5. 기준 자세 대비 joint delta와 J4 회전량을 제한하여 엉뚱한 IK branch를 거부한다.

### 고정 기준 pose preview 구현

`use_taught_slot0_place_reference:=true`일 때 기존 임의 orientation sampling을
우회하고 다음 고정 경로를 사용하도록 구현했다.

```text
pick retreat
 -> overview
 -> tray-view
 -> taught slot0 place reference
 -> preview hold 또는 명시 승인 시 gripper release
```

tray-view에서 기준 자세까지 J4 변화는 `175.94° -> 179.46°`로 작다. 따라서
기존 자동 후보에서 발생한 J4 급회전 branch를 사용하지 않는다.

첫 실기 검증 명령은 반드시 `execute_marker_place_release:=false`로 실행한다.

## Bringup YAML parsing incident

`bringup.launch.py` 순정 파일은 변경하지 않았다. 2026-06-11 실측 TCP link를
추가하면서 URDF XML 주석에 포함된 `center: 10mm` 문자열을 ROS Humble launch가
YAML mapping으로 오해해 bringup이 실패했다. 주석의 콜론만 제거하여 실측 TCP
구조를 유지한 채 launch parsing 문제를 해결했다.
