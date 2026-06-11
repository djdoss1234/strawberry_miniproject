# Claude Code Handoff - Measured TCP Transition

Date: 2026-06-11 KST

## 1. Immediate Objective

The current objective is to replace the legacy empirical tool-length compensation
with a measured physical grasp TCP, then safely revalidate the SW single-strawberry
harvest sequence.

Do not claim that measured-TCP robot execution or harvesting has succeeded yet.
The latest confirmed result is **plan-only success**.

## 2. Physical Measurement And Frame Convention

Measured geometry:

- robot flange to original gripper: approximately `160mm`
- original gripper to extension-part tip: approximately `110-120mm`
- flange to extension-part tip: approximately `270mm`
- intended grasp center: approximately `10mm` behind the tip
- flange to intended grasp center: approximately `260mm`

Current measured model:

- cuRobo `ee_link`: `grasp_tcp_link`
- `grasp_tcp_link`: fixed at TOOL `+Z 0.260m` from
  `gripper_rh_p12_rn_base`

Base axes while facing the whiteboard:

- base `+X`: right
- base `-X`: left
- base `+Y`: toward whiteboard
- base `-Y`: toward robot
- base `+Z`: up
- base `-Z`: down

Current horizontal grasp TOOL axes:

- TOOL `+Z`: toward whiteboard, final approach direction
- TOOL `-Z`: away from whiteboard, straight retreat direction
- TOOL `+X`: down
- TOOL `-X`: up
- TOOL `+Y`: left
- TOOL `-Y`: right

## 3. Why The Legacy Model Was A Problem

The previous model treated approximately `160mm` as the effective tool length,
although the physical grasp point is approximately `260mm` from the flange.
Runtime code compensated for the missing geometry with manual offsets and a
default extra advance.

That mixed:

- physical geometry error,
- perception/calibration error,
- and task-specific empirical correction.

It made approach depth difficult to explain and tune. The new measured profile
makes cuRobo plan directly for the measured grasp center and disables legacy
length compensation/default extra advance.

## 4. Implemented Changes

Important files:

- `config/curobo/e0509_gripper_measured_tcp.yml`
  - measured-TCP cuRobo profile
  - `ee_link: grasp_tcp_link`
- `config/curobo/e0509_gripper.urdf`
  - adds `grasp_tcp_link`
- `urdf/e0509_with_gripper.urdf.xacro`
  - adds matching link for ROS robot model
- `config/curobo/e0509_spheres.yml`
  - approximates extension-part collision volume
- `scripts/curobo_planner_node.py`
  - default profile: `measured_tcp_260mm`
  - rollback profile: `legacy_160mm`
  - default: `measured_tcp_plan_only=true`
  - measured mode disables legacy 160mm/manual extra-advance compensation
  - measured mode prepares a 60mm pre-approach and guarded 30mm TOOL +Z final move
- `docs/tool_geometry_measurement_20260611.md`
  - measurements, axes, experiments, and safety rules

Do not delete or revert unrelated/user changes. In particular,
`scripts/측정.py` is a user file and must not be modified or committed without
explicit confirmation.

## 5. Current Measured-TCP Motion Policy

```text
scan pose
 -> cuRobo plans measured grasp TCP to 60mm pre-approach
 -> Doosan MoveSplineJoint to pre-approach
 -> guarded Doosan MoveLine TOOL +Z 30mm
 -> gripper close
 -> BASE -Z detach pull
 -> Doosan MoveLine TOOL -Z 30mm
 -> return to scan pose
```

Why hybrid:

- plan-only experiments show 60mm pre-approach is reachable.
- endpoints closer to the wall repeatedly return IK_FAIL.
- some orientation branches cross the J4 wrap boundary and are rejected due
  to approximately `358deg` spline jumps.
- the final short, constrained straight segment is therefore reserved for a
  guarded vendor MoveLine.

## 6. Latest Experiment And Exact Interpretation

Latest run:

```text
logs/runtime/2026-06-11/
curobo_planner_node_20260611T153321-6fdb516e.jsonl
```

Result:

- target raw: `(-105,672,587)mm`
- grasp target after Z bias: `(-105,672,617)mm`
- first 3 orientation candidates: safely rejected for J4 spline jump
- next candidate: valid pre-approach plan
- pre-approach goal: `(-105,612,612)mm`
- planned end joints:
  `[120.0,-25.7,110.5,269.1,29.9,-1.1]deg`
- guarded final MoveLine prepared: `30mm`
- actual robot pick motion: **not dispatched**

This was not a planning failure. The robot returned home because the old
plan-only branch published `/dsr01/curobo/pick_complete`; the scan executor
mistook that for a completed pick.

That bug is now fixed:

- plan-only no longer publishes `pick_complete`
- runtime event is `measured_tcp_plan_only_hold`
- expected warning:
  `MEASURED_TCP_PLAN_ONLY_HOLD: /pick_complete was not published`

## 7. Next Exact Steps

### A. Rebuild and run plan-only again

```bash
cd ~/doosan_ws
colcon build --packages-select e0509_gripper_description
source install/setup.bash
ros2 run e0509_gripper_description curobo_planner_node.py
```

Expected:

- valid measured pre-approach plan
- guarded 30mm final MoveLine prepared
- no actual robot motion
- no `pick_complete`
- no automatic home return

Inspect the new JSONL and confirm:

```text
event=measured_tcp_plan_only_hold
pick_complete_published=false
```

### B. Only after A is confirmed, consider a first real execution

```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p measured_tcp_plan_only:=false
```

Real execution requirements:

- SW single target only
- clear surrounding leaves/fruit
- low speed
- E-stop ready
- observer watches flange, extension tip, and whiteboard clearance
- record video and JSONL

Stop immediately if TOOL +Z does not move toward the whiteboard or if the
physical tip/grasp center does not match the modeled endpoint.

### C. Rollback

```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p tool_model_profile:=legacy_160mm
```

Use rollback only as a reproducible baseline. Do not silently mix legacy
offsets with the measured profile.

## 8. Still Unverified / Known Risks

- measured `260mm` grasp center has not yet completed a real robot pick.
- the Doosan controller's configured TCP may not match `grasp_tcp_link`; verify
  TOOL MoveLine direction and distance separately.
- eye-in-hand transform and detected Y show drift; detections beyond the wall
  are currently clamped to `Y=672mm`.
- table/tray/place path remains a separate validation task.
- leaf/stem geometry is not represented in the cuRobo world.
- self-collision is currently disabled.
- gripper contact/current verification is implemented but hardware read
  reliability still requires repeated testing.
- `pick_complete` is still a legacy sequence-finished signal, not proof of
  successful harvest.

## 9. Safety And Data-Preservation Rules

- preserve calibration, model weights, runtime logs, and user-created files.
- never delete or reset dirty worktree changes.
- do not enable real measured-TCP execution before reviewing a fresh plan-only
  result.
- do not claim success without video/human label or reliable sensor evidence.
- after every important experiment, record:
  runtime JSONL path, parameters, target, observed motion, human result label,
  and any intervention.

## 10. Recommended Follow-Up Priorities

1. Verify plan-only hold behavior and no-home-return regression.
2. Verify Doosan TOOL axes with a safe 10mm jog/MoveLine.
3. Execute one measured-TCP SW approach at low speed.
4. Measure actual tip/grasp-center endpoint error.
5. Compare measured profile against `legacy_160mm`.
6. Continue automatic grasp verification and KPI collection.
7. Resume marker-based tray/place validation only after pick motion is stable.
