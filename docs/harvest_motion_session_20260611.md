# 수확 모션 세션 — 2026-06-11

## 세션 목표

measured TCP 260mm 프로필 실기 검증 및 파라미터 안정화.
SW 셀 단일 딸기 수확 모션 반복 성공 조건 확립.

---

## 1. 시작 상태

- `MEASURED_TCP_FINAL_STANDOFF_M = 0.030` → pre-approach 60mm + 직선 30mm = 90mm 총 진입
- `GRASP_Z_BIAS = 0.030`
- plan_only 검증 완료 (9159d74), 실기 실행으로 전환

---

## 2. 시도별 문제 및 수정

### 2-1. 그리퍼가 15cm 앞에서 닫힘

**원인**: standoff=+30mm → 총 진입 90mm → 줄기가 780~800mm 위치인데 TCP가 702mm에서 닫힘  
**수정**: `MEASURED_TCP_FINAL_STANDOFF_M = -0.120` → 총 진입 180mm → TCP 792mm 도달  
**결과**: 딸기 1개 파지 성공

### 2-2. 딸기마다 진입 거리 달라야 함

**원인**: Y-clamp가 raw 감지 Y를 672mm로 통일 → 모든 딸기의 TCP 종착점이 792mm로 고정.  
실제로는 앞으로 나온 딸기(raw_Y≈790mm)와 뒤에 있는 딸기(raw_Y≈835mm)의 실제 깊이가 다름.

**수정**: 클램핑 전 raw_Y 보존 → 딸기별 적응형 진입 거리 계산
```
detection_raw_y = float(p.y)  # 클램핑 전 원본
adaptive_dist = (detection_raw_y - Y_DETECTION_BIAS_M) - (WALL_SURFACE_Y_M - PRE_APPROACH_OFFSET)
final_approach_distance = max(baseline_180mm, min(adaptive_dist, 260mm))
```

- `Y_DETECTION_BIAS_M = 0.023` 시도 → 오히려 진입 거리를 줄여버려 실패 (808mm raw → 173mm, floor 180mm)
- `Y_DETECTION_BIAS_M = 0.000` 확정 → 808~815mm raw에서 196~203mm 진입

**결과**: 뒤에 있는 딸기까지 진입 거리 자동 조정, 다수 딸기 파지 가능

### 2-3. 맨 왼쪽 딸기(z=491mm)에서 잎 간섭

**원인**: 중앙 딸기(z=584mm)보다 93mm 낮은 위치. `GRASP_Z_BIAS=0`이면 그 높이의 잎이 수평 접근 경로를 막음.

**시도 A: 크레인 접근** (위에서 진입 후 하강)
```
cuRobo → (x, 612mm, z+80mm) → TOOL+Z 200mm → BASE -Z 80mm → gripper close
```
- `execute_base_z_relative()` 메서드 추가
- **문제**: BASE -Z 하강 시 주변 다른 줄기/잎을 쓸고 내려옴 → 간섭으로 정상 줄기도 움직임
- NW/NE 같은 밀도 높은 구역에서는 더 심해질 것 → 기각

**시도 B: GRASP_Z_BIAS 조정** (검증된 수평 접근 유지, Z 목표만 위로)
- `GRASP_Z_BIAS = 0.020` → KP0 기준 20mm 위를 파지 목표로 설정
- 잎 위에서 줄기 상단부를 파지 → BASE -Z 40mm 당기기로 분리
- **결과**: 맨 왼쪽 딸기 포함 정상 동작 확인

**크레인 코드는 보존** (`CRANE_Z_OFFSET_M = 0.000`으로 비활성화). 향후 perception 기반 장애물 등록과 함께 재활성화 검토.

### 2-4. GRASP_Z_BIAS 조정 이력

| 값 | 결과 |
|----|------|
| 0.030 | kp0보다 3~4cm 위 → 높이 과도 |
| 0.010 | 계획 단계에서 테스트 |
| 0.000 | 중앙 딸기 정확, 왼쪽 딸기 잎 간섭 |
| **0.020** | **현재 확정. 다수 딸기 파지 성공** |

---

## 3. 현재 확정 파라미터

```python
MEASURED_TCP_FINAL_STANDOFF_M = -0.120   # 기준 180mm 진입
Y_DETECTION_BIAS_M            =  0.000   # raw_Y 그대로 적응형 진입 계산
GRASP_Z_BIAS                  =  0.020   # KP0 +20mm 파지 목표
PRE_APPROACH_OFFSET           =  0.060   # 6cm pre-approach
CRANE_Z_OFFSET_M              =  0.000   # 크레인 비활성
DETACH_PULL_DOWN_MM           = 40.0     # BASE -Z 당기기
```

**적응형 진입 거리**: `final_approach_distance = max(180mm, min(raw_Y - 612mm, 260mm))`
- raw_Y=808mm → 196mm
- raw_Y=815mm → 203mm
- raw_Y=835mm → 223mm

---

## 4. 수확 시퀀스 (현재 기준)

```
scan pose
  ↓ cuRobo MoveSplineJoint
pre-approach (x, 612mm, z+20mm)    ← GRASP_Z_BIAS +20mm 반영
  ↓ TOOL +Z 196~220mm (raw_Y 기반 적응)
grasp position
  ↓ gripper close (pos 700)
  ↓ VERIFY_GRASP (서비스 미연결 → GRASP_UNVERIFIED)
  ↓ BASE -Z 40mm (detach pull)
  ↓ TOOL -Z 후퇴
scan pose 복귀
```

---

## 5. 미해결 / 다음 세션 작업

### 필수
- [ ] `VERIFY_GRASP` 서비스 연결 — 현재 `read_state service unavailable` 로 모든 시도가 `GRASP_UNVERIFIED`
- [ ] `label_harvest_attempt.py` 로 30회 시도 KPI 수집 시작
  - `measured_tcp_plan_only_hold` 를 `terminal_events` 에 추가 필요 (현재 누락)

### 중기
- [ ] Place 시퀀스 검증 (marker 기반 tray 배치)
- [ ] 비전 타겟 일관성 확인 (x=-345 vs -401mm 편차 원인)
- [ ] `GRIPPER_LEN` 실측 재확인 (160mm vs 실제 파지 홈 위치)

### 장기 (NW/NE 대응)
- [ ] point cloud 기반 잎/줄기 동적 장애물 등록 → cuRobo 경로 계획 정확도 향상
  - 현재 `Leaf/stem geometry is not in the cuRobo world` 경고 항상 출력
  - 크레인 접근(`CRANE_Z_OFFSET_M`) 활성화는 이 작업과 함께 재검토
- [ ] NW/NE 셀 스캔 및 수확 모션 파라미터 재조정

---

## 6. 알려진 한계

- `GRASP_UNVERIFIED`: 그리퍼 위치/전류 read_state 서비스 미연결 상태. 성공 여부는 사람 관찰로만 판단.
- Y-clamp (672mm): FK calibration drift 미보정. raw_Y 적응 진입으로 보상 중이나 근본 해결 아님.
- 잎/줄기 geometry 없음: 밀도 높은 구역에서 접근 경로 간섭 예측 불가.
- SW 셀 기준 파라미터: NW/NE는 별도 튜닝 필요.
