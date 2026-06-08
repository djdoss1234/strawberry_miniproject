# Harvest Test Strategy - 2026-06-04

## Goal

Move from scan-pose/debug work into staged harvest validation.

The near-term objective is not full autonomous harvesting over all cells.  The immediate objective is to make one conservative rule-based harvest succeed from the easiest scene, then increase difficulty step by step.

## Scene Difficulty By Root Cell

Current physical board setup:

| Cell | Scene Role | Difficulty | Purpose |
| --- | --- | --- | --- |
| `root/sw` | Single strawberry | Easy | First real grasp target. Validate far-view lock, close-confirm, direct grasp, retreat. |
| `root/ne` | Clustered strawberries | Medium | Test target selection, neighbor filtering, target lock, and avoiding nearby fruits. |
| `root/nw` | Stem/leaf occlusion | Hard | Test occlusion handling and decide what should be handed to VLA/reobserve policy. |
| `root/se` | Empty cell | Control | Verify empty-cell behavior and no false harvest attempt. |

Recommended validation order:

```text
root/sw -> root/ne -> root/nw
```

`root/se` should be used as a negative/control check, not as a harvest target.

## Current Rule-Based Harvest Policy

Use the far scan view to decide the target.

```text
far view = maturity decision + target lock
close view = geometry confirmation only
```

Important rule:

Do not let the close view freely change `ripe/unripe/sick` classification.  Close-up testing showed that maturity classification often collapses toward `ripe` due to close-range domain shift, reflection, gripper occlusion, and overlapping leaves/fruits.

## SW First-Test Sequence

For `root/sw`:

1. Move to `root/sw` scan/grasp-ready pose.
2. Wait for far-view fusion detection.
3. Lock one stable ripe target.
4. Move to close-confirm pose generated from the locked target.
5. At close-confirm:
   - ignore new maturity class changes
   - require fresh fused pick pose near the locked target
   - require KP/depth geometry to still be valid
6. If confirmed, run direct grasp.
7. Close gripper and retreat.
8. Do not place into egg tray yet.

Current place status:

```text
ENABLE_PLACE_SEQUENCE = False
```

Reason: table/tray collision risk is unresolved.

## Escalation Policy

If a target is not safely harvestable by the current rule-based policy:

```text
skip / mark failed / hand to future VLA
```

Do not keep adding manual poses for every failure.

Failures that should become VLA/reobserve cases:

- stem hidden by leaf
- strawberry hidden behind another fruit
- close-confirm cannot preserve target lock
- class becomes unstable at close range
- gripper blocks the camera view
- approach would require unsafe joint branch or collision-prone motion
- all three stem keypoints are not visible with sufficient confidence
- stem keypoint depth or 3-D segment geometry is implausible

Suggested result codes:

```text
CLASS_UNSTABLE_CLOSE_VIEW
TARGET_LOCK_LOST_CLOSE_VIEW
KP_LOST_CLOSE_VIEW
OCCLUDED_REOBSERVE_REQUIRED
VLA_REQUIRED
EMPTY_CELL_CONFIRMED
```

## Notes For Documentation

This staged order is intentional:

1. `root/sw` validates the core grasp pipeline on a simple single target.
2. `root/ne` adds clustered-fruit ambiguity after the single-target baseline works.
3. `root/nw` represents hard occlusion and should not be forced with brittle rules.
4. `root/se` verifies the system does not harvest from an empty cell.

This structure is useful for the team Notion page and portfolio because it separates:

- perception capability
- motion/grasp capability
- cluster handling
- occlusion/VLA handoff
- empty-cell false-positive behavior
