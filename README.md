# avoidance_sim_ws

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim Harmonic용 직선도로 LiDAR 인지·경로 기록·2 m 정지 및 명시적 승인 기반 회피 주행 환경이다. 모든 ROS 실행은 CycloneDDS와 `ROS_DOMAIN_ID=12`를 사용한다.

## 의존성과 빌드

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-xacro ros-jazzy-sensor-msgs-py python3-yaml
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 터미널별 실행 순서

터미널 1 — 장애물 없는 Gazebo, LiDAR 처리, RViz2 및 실제 odometry 기록:

```bash
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch avoidance_gazebo straight_avoidance.launch.py \
  spawn_obstacles:=false use_rviz:=true record_route:=true \
  route_file:=/home/qor/avoidance_sim_ws/routes/straight_reference.csv
```

터미널 2 — 수동 주행(`w/s`: 전후진, `a/d`: 좌우, `space`: 정지, `q`: 종료):

```bash
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run avoidance_route manual_teleop
```

기존 파일을 덮어쓰지 않으며 같은 이름이 있으면 timestamp가 붙은 새 CSV를 만든다. 기록과 시뮬레이션은 각 터미널에서 `Ctrl+C`로 정상 종료한다.

장애물 검출 테스트:

```bash
source /opt/ros/jazzy/setup.bash
source ~/avoidance_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch avoidance_gazebo straight_avoidance.launch.py spawn_obstacles:=true use_rviz:=true
```

RViz만 다시 실행하려면:

```bash
source /opt/ros/jazzy/setup.bash
source ~/avoidance_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2 -d $(ros2 pkg prefix avoidance_lidar)/share/avoidance_lidar/rviz/avoidance_lidar.rviz
```

## 주요 토픽

| 토픽 | 타입 |
|---|---|
| `/scan_front`, `/scan_rear` | `sensor_msgs/msg/LaserScan` |
| `/odom` | `nav_msgs/msg/Odometry` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` |
| `/avoidance/lidar/roi_points` | `sensor_msgs/msg/PointCloud2` |
| `/avoidance/lidar/roi_marker` | `visualization_msgs/msg/Marker` |
| `/avoidance/lidar/obstacles` | `visualization_msgs/msg/MarkerArray` |
| `/avoidance/lidar/nearest_point` | `geometry_msgs/msg/PointStamped` |
| `/avoidance/lidar/nearest_distance` | `std_msgs/msg/Float32` |
| `/avoidance/lidar/obstacle_detected`, `/avoidance/lidar/valid` | `std_msgs/msg/Bool` |
| `/avoidance/route/live_path`, `/avoidance/route/saved_path`, `/avoidance/route/csv_path` | `nav_msgs/msg/Path` |

기본 CSV 위치는 `~/avoidance_sim_ws/routes/straight_reference.csv`이며 거리·주기 조건은 `avoidance_route/config/route_recorder.yaml`, ROI·클러스터 조건은 `avoidance_lidar/config/front_lidar.yaml`에서 바꾼다.

## 장애물 없는 CSV 재현 주행

`route_follower`만 `/cmd_vel` 제어권을 가져야 하므로 재현 중에는 `manual_teleop`을 실행하지 않는다. Pure Pursuit 내부 조향각은 radian이며, `/cmd_vel.angular.z`에는 Gazebo Ackermann 플러그인이 요구하는 yaw rate(rad/s)를 발행한다.

터미널 1 — 시뮬레이션, follower, LiDAR 및 RViz2:

```bash
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch avoidance_gazebo route_replay.launch.py \
  route_file:=/home/qor/avoidance_sim_ws/routes/straight_reference.csv \
  spawn_obstacles:=false use_rviz:=true auto_start:=false
```

터미널 2 — 상태 확인:

```bash
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /avoidance/route/status
```

터미널 3 — 명시적 출발 또는 비상 정지:

```bash
source /opt/ros/jazzy/setup.bash
source ~/avoidance_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service call /avoidance/route/start std_srvs/srv/Trigger "{}"
ros2 service call /avoidance/route/stop std_srvs/srv/Trigger "{}"
```

기본 설정은 `avoidance_route/config/route_follower.yaml`이다. 기준 경로는 `/avoidance/route/reference_path`, 실제 궤적은 `/avoidance/route/actual_path`, 최종 측정값은 `/avoidance/route/metrics`에서 확인한다. 실제 궤적 CSV는 `routes/replay_actual*.csv`에 기록되며 원본 CSV는 수정하지 않는다.

## 2 m LiDAR 회피 경로 생성 및 승인 후 주행

LiDAR 점을 scan timestamp의 TF로 `odom`에 변환해 벽과 추적 장애물을 분류한다. CSV footprint 충돌이 예상되고 LiDAR 원점에서 물리 장애물 표면까지 2.0 m 이하가 3회 확인되면 follower가 정지한다. 정지 확인 후 planner가 quintic 후보를 생성해 충돌·clearance·`±20°`를 검사한다. 차량은 `PATH_READY`에서도 정지하며 `/avoidance/start_selected_path` 호출 후에만 회피 경로를 주행한다. 회피 종료 때 한 제어 주기 정지한 다음 진행 index를 역행하지 않고 CSV로 복귀한다. `/cmd_vel`은 항상 `route_follower` 하나만 발행한다.

터미널 1 — 랜덤 장애물, follower, planner, Gazebo 및 RViz2:

```bash
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch avoidance_gazebo avoidance_planning.launch.py \
  route_file:=/home/qor/avoidance_sim_ws/routes/straight_reference.csv \
  spawn_obstacles:=true use_rviz:=true auto_start:=false planner_enabled:=true \
  obstacle_seed:=-1 replan_trigger_distance_m:=2.0 max_steering_deg:=20.0
```

터미널 2 — planner 상태:

```bash
source /opt/ros/jazzy/setup.bash
source ~/avoidance_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /avoidance/planner_status
```

터미널 3 — 재계획 latch와 명시적 출발/정지:

```bash
source /opt/ros/jazzy/setup.bash
source ~/avoidance_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /avoidance/replan_required
ros2 service call /avoidance/route/start std_srvs/srv/Trigger "{}"
ros2 service call /avoidance/start_selected_path std_srvs/srv/Trigger "{}"
ros2 service call /avoidance/stop std_srvs/srv/Trigger "{}"
```

GPS 주행 계약은 `/gps_drive=2.00`과 `/gps_wheel`, 회피 주행 계약은 `/lidar_drive=1.00`과 `/lidar_wheel`이다. 정지 때 두 쌍 모두 0이며 `/avoidance/control_source`는 `GPS`, `LIDAR`, `STOP` 중 하나다. 선택/실제 회피 경로는 `/avoidance/selected_path`, `/avoidance/actual_avoidance_path`에서 확인한다. `obstacle_seed:=-1`은 시스템 난수와 직전 배치 비교를 사용하고, 0 이상은 재현 테스트 전용이다. 설정은 `avoidance_planner/config/avoidance_planner.yaml`에 있다.
