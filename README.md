# 실차 RPLIDAR 장애물 우회 시스템

## 1. 프로젝트 목적

ROS 2 Jazzy에서 실제 RPLIDAR의 `/scan_front`를 받아 장애물을 검출·추적하고, 기존 참조 경로를 기준으로 충돌 없는 Frenet 우회 경로를 생성·추종한 뒤 원래 경로에 재합류한다. 최종 차량 명령은 `/lidar_drive`와 `/lidar_wheel`로만 출력한다.

## 2. 전체 구조

```text
RPLIDAR → /scan_front → avoidance_lidar
                         ├─ lidar_safety → 곡선 ROI/TTC/즉시 정지/watchdog
                         └─ avoidance_planner → Frenet 우회 경로

/avoidance/route/reference_path → route_follower
                                  ↓
                    /avoidance/command/*_requested
                                  ↓
                            lidar_safety
                                  ↓
                    /lidar_drive, /lidar_wheel
```

`avoidance_planner`가 어디로 피할지 결정하고, `lidar_safety`는 현재 명령을 통과시켜도 안전한지만 결정한다. 최종 두 제어 토픽은 `lidar_safety` 한 노드만 발행한다.

## 3. 필요한 하드웨어

- RPLIDAR A1/A2/A3/S1/S2/S3/T1 계열과 USB 직렬 연결
- ROS 2가 실행되는 Ubuntu 24.04 컴퓨터
- `/odom`과 `odom → base_link` TF를 제공하는 실차 위치 추정 계층
- 다음 계약을 처리하는 차량 MCU
  - `/lidar_drive`: `std_msgs/msg/Float32`, MCU discrete drive level
  - `/lidar_wheel`: `std_msgs/msg/Int32`, 조향각(도), 범위 `-27..+27`

기본 드라이버 설정은 A2M12 기준 `/dev/ttyUSB0`, 256000 baud이다. A1은 일반적으로 115200 baud를 사용하므로 launch argument로 변경한다.

## 4. ROS 2 Jazzy 환경

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 5. 빌드 방법

```bash
cd /home/qor/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 6. RPLIDAR 실행

```bash
ros2 launch avoidance_lidar lidar_driver.launch.py \
  serial_port:=/dev/ttyUSB0 \
  serial_baudrate:=256000 \
  lidar_frame:=front_laser \
  scan_mode:=Sensitivity
```

확인:

```bash
ros2 topic info /scan_front
ros2 topic hz /scan_front
```

센서 장착 위치 기본값은 `base_link` 기준 `x=0.65`, `y=0.0`, `z=0.20`, `yaw=0.0`이다. 실제 장착 치수를 launch argument `lidar_x`, `lidar_y`, `lidar_z`, `lidar_yaw`로 반드시 맞춘다.

## 7. 우회 스택 실행

외부 GPS·카메라 경로가 `nav_msgs/msg/Path`를 `/avoidance/route/reference_path`로 발행하는 경우:

```bash
ros2 launch avoidance_lidar avoidance_core.launch.py use_rviz:=true
```

기존 실차 주행 CSV를 사용할 경우:

```bash
ros2 launch avoidance_lidar avoidance_core.launch.py \
  route_file:=/absolute/path/to/vehicle_route.csv \
  use_rviz:=true
```

경로 frame은 기본적으로 `odom`이어야 한다. 출발 전 `/avoidance/route/start` 서비스를 한 번 호출한다.

```bash
ros2 service call /avoidance/route/start std_srvs/srv/Trigger '{}'
```

## 8. 전체 실차 실행

```bash
ros2 launch avoidance_lidar avoidance_vehicle.launch.py \
  serial_port:=/dev/ttyUSB0 \
  serial_baudrate:=256000 \
  route_file:=/absolute/path/to/vehicle_route.csv \
  use_rviz:=true
```

## 9. 입력 토픽

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/scan_front` | `sensor_msgs/msg/LaserScan` | 전방 RPLIDAR |
| `/odom` | `nav_msgs/msg/Odometry` | 차량 위치·속도 |
| `/avoidance/route/reference_path` | `nav_msgs/msg/Path` | GPS·카메라·상위 planner 참조 경로 |

후방 LiDAR는 현재 기본 우회 흐름에서 요구하지 않는다. 필요할 때만 `rear_lidar_required` 설정을 활성화한다.

## 10. 출력 토픽

| 토픽 | 타입 | 단위/범위 |
|---|---|---|
| `/lidar_drive` | `std_msgs/msg/Float32` | MCU discrete drive level |
| `/lidar_wheel` | `std_msgs/msg/Int32` | 조향각(도), `-27..+27` |
| `/avoidance/selected_path` | `nav_msgs/msg/Path` | 선택된 우회 경로 |
| `/avoidance/safety/stop_required` | `std_msgs/msg/Bool` | 안전 계층 정지 상태 |
| `/avoidance/safety/status` | `std_msgs/msg/String` | scan·명령 watchdog 진단 |

참조 경로 추종은 기본 drive level 2, 우회 경로 추종은 level 1을 요청한다. 알고리즘 내부 m/s 값은 Pure Pursuit와 동역학 제한 계산에만 사용되며 `/lidar_drive`에 m/s로 발행되지 않는다.

## 11. 주요 파라미터

- `avoidance_lidar/config/rplidar.yaml`: serial port, baudrate, frame, scan mode
- `avoidance_lidar/config/front_lidar.yaml`: 전방 ROI, scan 범위, clustering
- `avoidance_lidar/config/safety.yaml`: 조향 경로형 ROI, 속도별 제동거리, TTC, scan/command timeout, 조향 한계
- `avoidance_planner/config/avoidance_planner.yaml`: 차량 폭·길이·wheelbase, clearance, 재계획 거리, Frenet 후보
- `avoidance_route/config/route_follower.yaml`: lookahead, 속도, 재합류, drive level

차량 형상과 LiDAR 장착 위치는 실측값을 사용한다. 기존 기본값을 검증 없이 변경하지 않는다.

## 12. RViz에서 우회 경로 확인

`use_rviz:=true`로 실행한 뒤 Fixed Frame을 `odom`으로 둔다. 다음 display를 확인한다.

- Front LaserScan
- Replay Reference Path
- Candidate Avoidance Paths
- Selected Avoidance Path
- Static/Dynamic Obstacles
- Safe Corridor
- Actual Avoidance Path

## 13. 실차 테스트 순서

1. 바퀴를 지면에서 띄우거나 구동 전원을 차단한 상태에서 시작한다.
2. `ROS_DOMAIN_ID=12`와 CycloneDDS를 설정한다.
3. LiDAR만 실행해 `/scan_front` 타입·주기·frame과 TF를 확인한다.
4. 우회 core를 실행하고 `/lidar_drive`, `/lidar_wheel` publisher가 각각 `lidar_safety` 하나인지 확인한다.
5. 참조 경로와 `/odom`을 공급한다.
6. 장애물을 정지 거리 안팎으로 이동해 hysteresis, TTC, scan timeout 정지를 확인한다.
7. 저속·넓은 폐쇄 구역에서 경로 추종, 우회, 재합류를 순서대로 확인한다.
8. 이상 시 즉시 다음 서비스를 호출한다.

```bash
ros2 service call /avoidance/stop std_srvs/srv/Trigger '{}'
```

## 14. 안전 주의사항

- 첫 시험은 반드시 구동륜을 띄우고 수행한다.
- 물리 비상 정지 장치를 별도로 준비한다.
- `/scan_front`, `/odom`, TF가 끊기면 차량이 정지하는지 매 시험 전에 확인한다.
- 안전 계층은 장애물 회피 경로를 만들지 않으며 제동 하드웨어를 대체하지 않는다.
- 실제 MCU의 drive level 의미와 조향 부호를 확인한 뒤 차량을 지면에서 주행시킨다.

## 15. 현재 지원 범위

- 전방 RPLIDAR 한 대
- 정적·동적 장애물 추적
- Frenet lateral offset 및 quintic trajectory 후보
- 차량 전체 footprint 충돌 검사와 curvature/조향 검증
- STOP → PLAN → AVOID → REJOIN 흐름
- CSV 또는 외부 `nav_msgs/msg/Path` 참조 경로
- 조향 경로형 ROI와 속도 기반 ROI·제동거리
- 상대 접근 추세 기반 TTC 정지와 STOP/SLOW/CAUTION 진단
- scan/command watchdog, invalid scan fail-safe, 조향 포화

## 16. 제외 범위

주차 기능은 현재 포함하지 않는다. 슬롯 검출·저장·선택과 주차 경로 생성 노드도 이 워크스페이스에 포함하지 않는다.
