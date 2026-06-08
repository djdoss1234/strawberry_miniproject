# Tray Localization Note - 2026-06-04

## Purpose

Record the egg-tray view pose and the first ArUco-based 15-slot localization checks for later place-sequence recovery.

## Egg-Tray View Pose

This pose sees the egg tray and its ArUco marker. Use it as the camera pose for tray re-localization before generating place targets.

```text
joints_deg = [-0.02, -2.41, 111.87, 175.94, -31.34, 93.42]
```

Source location:

```text
/home/user/Downloads/share_tray/robot_poses.yaml
egg_tray_view.joints_deg
```

Notes:

- This is not the home/overview pose.
- It is intended for `share_tray` ArUco localization and later place target generation.
- `task_pose` is still unset in `robot_poses.yaml`; the validated value is the joint pose above.

## Localization Outputs

Two ArUco localization runs were generated in `~/Downloads/share_tray/output/`.

First run:

```text
tray_cells_20260604_151803.json
robot_poses_updated_20260604_151803.yaml
```

After moving the egg tray slightly:

```text
tray_cells_20260604_152536.json
robot_poses_updated_20260604_152536.yaml
```

The second run confirmed that the tray pose is recomputed from the marker rather than reusing fixed taught coordinates.

## Geometry Check

For the moved-tray run, the generated 15 slot centers preserved the configured tray pitch:

```text
col pitch avg/min/max = 51.7 / 51.7 / 51.7 mm
row pitch avg/min/max = 50.0 / 50.0 / 50.0 mm
```

Representative contact targets from `tray_cells_20260604_152536.json`:

```text
cell 0  row=0 col=0 contact=(451.3, -294.8, 628.2) mm
cell 14 row=4 col=2 contact=(501.9, -388.0, 429.7) mm
```

## Coordinate Meaning

`position_tray_plane_mm`:
Slot center on the tray plane.

`position_contact_mm`:
Fingertip/extension contact target, currently tray plane plus 60 mm standoff.

`position_tcp_mm`:
Robot TCP target after applying the 120 mm gripper extension compensation.

For real place execution, validate `position_tcp_mm` against table/tray collision before enabling automatic place.

## Safety Status

Marker-derived place integration was added on 2026-06-08, but release remains
explicitly gated. The first physical validation must run in preview mode and stop
at the marker-derived slot above pose. Table/tray collision geometry is still
not active in the cuRobo world, so preview clearance inspection and low-speed
single-slot validation are required before enabling release.
