# Close-Range Perception Limit - 2026-06-04

## Observation

During harvest testing, the fusion detector behaves differently depending on viewing distance.

At the taught scan distance, the two-model fusion is reasonably useful:

- ripe / unripe / sick classes are separated fairly well
- stem keypoints are visible enough to form a pick candidate
- target selection is stable enough for scan-time candidate discovery

When the gripper moves closer, the same physical strawberries often become unstable:

- class labels tend to collapse toward `ripe`
- detections flicker between nearby fruits and leaves
- pose keypoints and segmentation masks do not always correspond to the same fruit
- this can happen even when the robot is stopped, so it is not just motion blur

Representative stable far-view pose:

```text
joints_deg = [144.09, 22.90, -1.00, -238.52, -75.31, 108.68]
```

## Current Diagnosis

This is primarily a perception/domain issue, not a robot execution issue.

Likely causes:

- close-up images contain larger occlusions from the gripper, leaves, and neighboring fruit
- the training data likely has fewer extreme close-up examples with the 15.8 cm gripper extension visible
- close-up fruit texture/lighting differs from the scan-view distribution
- segmentation and pose models run independently, so close overlap can cause mask/keypoint association errors
- RealSense depth and RGB alignment become more sensitive near object boundaries

## Mitigation Already Added

The fusion node now uses stricter target stabilization:

```text
commit a147177 fix: stabilize close-range fusion matching
commit 82a31df fix: lock fusion target during close-up detection
```

Implemented changes:

- match pose to seg mask using KP0/KP1 evidence, not only pose bbox center
- require more stable hits before publishing
- publish only one locked target instead of every stable ripe track
- keep a nearby target lock through short detector flicker
- expose target lock parameters:
  - `target_lock_enabled`
  - `target_lock_ttl_sec`
  - `target_switch_distance_m`

This reduces target switching, but it does not fully solve wrong maturity classification at close range.

## Operational Rule For Now

Use close-up perception as a confirmation aid only, not as the main maturity classifier.

Recommended rule-based flow:

1. Classify and select candidates from the farther scan/gripper-centered view.
2. Lock one target ID before moving closer.
3. During close approach, keep the locked target unless it is clearly lost.
4. Do not reclassify an unripe/sick/ripe decision from scratch at the closest view.
5. If the close view reports a different class but the far-view target was stable, treat it as `CLASS_UNSTABLE_CLOSE_VIEW`.
6. Abort or reobserve if the locked target disappears or the stem keypoint becomes invalid.

## Logging Recommendation

For each pick attempt, record both views:

```text
far_scan_image
far_seg_class
far_pose_keypoints
close_confirm_image
close_seg_class
close_pose_keypoints
class_changed_at_close_range: true/false
result_code
```

Suggested failure/result code:

```text
CLASS_UNSTABLE_CLOSE_VIEW
TARGET_LOCK_LOST_CLOSE_VIEW
KP_LOST_CLOSE_VIEW
```

## Next Improvement

The robust solution is model/data improvement plus task-level hysteresis:

- add close-up training images from the actual gripper-mounted camera
- include the 15.8 cm gripper extension, leaves, neighboring fruits, and whiteboard background in the dataset
- label close-up ripe/unripe/sick examples separately
- train/validate with a distance split: scan-distance vs close-grasp-distance
- keep the harvest state machine from changing target identity or maturity class during final approach

Until that dataset exists, the safest policy is:

```text
far view decides what to harvest;
close view verifies geometry/stem visibility;
close view must not freely change the selected fruit class.
```
