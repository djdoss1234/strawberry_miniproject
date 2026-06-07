# RealSense + YOLO + cuRobo Strawberry Harvest System

```mermaid
flowchart LR
    subgraph HW["Hardware"]
        CAM["Intel RealSense RGB-D<br/>eye-in-hand camera"]
        ROBOT["Doosan E0509<br/>robot arm"]
        GRIPPER["ROBOTIS RH-P12-RN<br/>gripper"]
        TRAY["Egg tray / basket<br/>place slots"]
        BOARD["Whiteboard wall<br/>strawberry fixture"]
    end

    subgraph PERCEPTION["Perception Node<br/>scripts/strawberry_yolo_node.py"]
        RGBD["RGB + aligned depth frames"]
        YOLO["YOLO strawberry detection<br/>ripe / unripe"]
        FILTER["Ripe filter<br/>HSV red ratio + saturation"]
        DEPTH["Depth patch filtering<br/>valid depth / near surface"]
        TF["Hand-eye transform<br/>camera -> gripper -> base"]
        TARGET["Pick target PoseStamped<br/>/dsr01/curobo/pick_pose"]
    end

    subgraph PLANNING["Planning & Sequencing Node<br/>scripts/curobo_planner_node.py"]
        STATE["/dsr01/joint_states<br/>current joints"]
        WORLD["cuRobo world model<br/>whiteboard / demo collision setup"]
        CUROBO["cuRobo MotionGen<br/>pre-approach / grasp endpoint validation / retreat"]
        SEQ["Harvest sequence<br/>pre-approach -> stop -> straight advance -> close -> retreat"]
        SLOTS["config/place_slots.yaml<br/>slot above / release joints"]
        SAFETY["Safety heuristics<br/>J1 branch check / left-safe transfer / offsets"]
    end

    subgraph EXEC["Robot Execution"]
        SPLINE["Doosan MoveSplineJoint<br/>/dsr01/motion/move_spline_joint"]
        MOVEL["Doosan MoveLine<br/>/dsr01/motion/move_line<br/>slow TOOL +Z final grasp advance"]
        MOVEJ["Doosan MoveJoint<br/>/dsr01/motion/move_joint<br/>short place/home moves"]
        OPEN["/dsr01/gripper/open"]
        POSCMD["/dsr01/gripper/position_cmd<br/>soft close steps"]
        DONE["/dsr01/curobo/pick_complete"]
    end

    subgraph TOOLS["Teaching / Debug Tools"]
        TEACH["teach_place_slots.py<br/>save slot above/release"]
        JOG["joint_jog_control.py<br/>manual joint step control"]
        TCP["/dsr01/aux_control/get_current_posx<br/>DART TCP verification"]
        RVIZ["MoveIt / RViz<br/>URDF + environment visualization"]
    end

    CAM --> RGBD
    RGBD --> YOLO
    YOLO --> FILTER
    FILTER --> DEPTH
    DEPTH --> TF
    TF --> TARGET

    TARGET --> SEQ
    STATE --> CUROBO
    WORLD --> CUROBO
    SLOTS --> SEQ
    SAFETY --> SEQ
    SEQ --> CUROBO
    CUROBO --> SPLINE
    SEQ --> MOVEL
    SEQ --> MOVEJ
    SEQ --> OPEN
    SEQ --> POSCMD
    SEQ --> DONE

    SPLINE --> ROBOT
    MOVEL --> ROBOT
    MOVEJ --> ROBOT
    OPEN --> GRIPPER
    POSCMD --> GRIPPER
    ROBOT --> GRIPPER
    GRIPPER --> BOARD
    GRIPPER --> TRAY

    TEACH --> SLOTS
    JOG --> ROBOT
    TCP --> TEACH
    RVIZ --> WORLD
```

## Runtime Pipeline

```mermaid
sequenceDiagram
    participant Camera as RealSense
    participant Vision as strawberry_yolo_node
    participant Planner as curobo_planner_node
    participant CuRobo as cuRobo MotionGen
    participant Doosan as Doosan ROS2 Services
    participant Gripper as RH-P12-RN Gripper

    Camera->>Vision: RGB + aligned depth
    Vision->>Vision: YOLO detection + ripe filter
    Vision->>Vision: depth patch -> 3D point
    Vision->>Vision: camera -> gripper -> base transform
    Vision->>Planner: /dsr01/curobo/pick_pose

    Planner->>CuRobo: plan pre-approach and validate grasp endpoint
    CuRobo-->>Planner: safe pre-approach trajectory + validated grasp pose
    Planner->>Doosan: MoveSplineJoint

    Planner->>Planner: stop and settle at pre-approach
    Planner->>Doosan: MoveLine relative TOOL +Z at low speed
    Planner->>Planner: stop and settle at grasp pose
    Planner->>Gripper: soft close position_cmd

    Planner->>CuRobo: plan retreat / safe transfer
    CuRobo-->>Planner: joint trajectory
    Planner->>Doosan: MoveSplineJoint

    Planner->>Planner: select place slot
    Planner->>CuRobo: plan to slot above
    CuRobo-->>Planner: joint trajectory
    Planner->>Doosan: MoveSplineJoint / MoveJoint
    Planner->>Gripper: open at release pose
    Planner->>Doosan: return home
    Planner-->>Vision: /dsr01/curobo/pick_complete
```

## Current Harvest Motion Notes

2026-06-07 기준 최종 파지 접근은 hybrid 방식이다.

- cuRobo는 pre-approach 경로와 grasp endpoint의 IK/collision/branch 안전성을 검증한다.
- 실제 마지막 진입은 pre-approach에서 완전히 멈춘 뒤 Doosan `MoveLine`으로
  TOOL `+Z` 방향 저속 직선 이동한다.
- 현재 SW 실기에서 수평 정면 진입 방향은 확인했지만, 최종 진입 깊이가 부족해
  실제 줄기 파지는 아직 성공하지 못했다.
- `grasp OK`와 `pick_complete`는 실제 파지 성공을 의미하지 않는다.

상세 기록: [harvest_motion_session_20260607.md](harvest_motion_session_20260607.md)
