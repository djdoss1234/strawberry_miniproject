# E0509 + RH-P12-RN-A Gripper Description

Doosan E0509 로봇팔과 ROBOTIS RH-P12-RN-A 그리퍼를 결합한 ROS2 패키지

## 개요

이 패키지는 Doosan E0509 6축 로봇팔에 ROBOTIS RH-P12-RN-A 그리퍼를 장착한 통합 로봇 시스템을 위한 URDF, launch 파일, 그리퍼 컨트롤러를 제공합니다.

## 특징

- ✅ E0509 + 그리퍼 결합 URDF/XACRO
- ✅ Doosan Virtual Robot (에뮬레이터) 지원
- ✅ ros2_control 기반 조인트 제어
- ✅ 그리퍼 stroke 기반 제어 (DART Platform 호환)
- ✅ RViz 시각화

## 의존성

- ROS2 Humble
- Gazebo Fortress (Ignition Gazebo 6)
- [doosan-robot2](https://github.com/doosan-robotics/doosan-robot2)
- [RH-P12-RN-A](https://github.com/ROBOTIS-GIT/RH-P12-RN-A)

## 설치
```bash
# 워크스페이스 생성
mkdir -p ~/doosan_ws/src
cd ~/doosan_ws/src

# 의존 패키지 클론
git clone https://github.com/doosan-robotics/doosan-robot2.git
git clone https://github.com/ROBOTIS-GIT/RH-P12-RN-A.git

# 이 패키지 클론
git clone https://github.com/fhekwn549/e0509_gripper_description.git

# 빌드
cd ~/doosan_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 사용법

### 1. RViz 시각화 (조인트 슬라이더)
```bash
ros2 launch e0509_gripper_description display.launch.py
```

### 2. Virtual Robot 실행 (에뮬레이터)
```bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=virtual host:=127.0.0.1 port:=12345
```

### 3. 로봇 제어
```bash
# 조인트 이동
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [30.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 30.0}"

# 홈 위치
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: 30.0, acc: 30.0}"
```

### 4. 그리퍼 제어
```bash
# 그리퍼 열기
ros2 service call /dsr01/gripper/open std_srvs/srv/Trigger

# 그리퍼 닫기
ros2 service call /dsr01/gripper/close std_srvs/srv/Trigger

# Stroke 값으로 제어 (0=열림, 700=완전히 닫힘)
ros2 topic pub /dsr01/gripper/stroke std_msgs/msg/Int32 "{data: 350}" --once
```

## 파일 구조
```
e0509_gripper_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── urdf/
│   └── e0509_with_gripper.urdf.xacro
├── launch/
│   ├── display.launch.py          # RViz 시각화
│   └── bringup.launch.py          # Virtual/Real 로봇 실행
├── scripts/
│   └── gripper_joint_publisher.py # 그리퍼 컨트롤러
└── rviz/
    └── display.rviz
```

## 그리퍼 제어 인터페이스

| 인터페이스 | 타입 | 설명 |
|-----------|------|------|
| `/dsr01/gripper/open` | Service (Trigger) | 그리퍼 열기 |
| `/dsr01/gripper/close` | Service (Trigger) | 그리퍼 닫기 |
| `/dsr01/gripper/stroke` | Topic (Int32) | Stroke 값 (0~700) |

## 환경

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Fortress

## License

Apache-2.0

## Author

fhekwn549
