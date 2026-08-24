# avoidance_sim_ws

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim Harmonic용 직선·S자 도로 LiDAR 인지·경로 기록·2 m 정지 및 자동 반복 회피 주행 환경이다. 선택한 기준 CSV를 최초 서비스 호출 한 번으로 끝까지 주행하며, 장애물을 만날 때마다 정지·회피 경로 생성·자동 회피 주행·CSV 복귀를 서비스 재호출 없이 반복한다. 모든 ROS 실행은 CycloneDDS와 `ROS_DOMAIN_ID=12`를 사용한다.

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

기존 파일을 덮어쓰지 않으며 같은 이름이 있으면 timestamp가 붙은 새 CSV를 만든다. 실제 주행 로그는 `runtime_logs/` 또는 `verification_logs/`에 저장하고 `routes/`에는 저장하지 않는다. 기록과 시뮬레이션은 각 터미널에서 `Ctrl+C`로 정상 종료한다.

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

기본 설정은 `avoidance_route/config/route_follower.yaml`이다. 기준 경로는 `/avoidance/route/reference_path`, 실제 궤적은 `/avoidance/route/actual_path`, 최종 측정값은 `/avoidance/route/metrics`에서 확인한다. 실제 궤적 CSV 기록은 기본 비활성화되며, 필요할 때만 `actual_path_csv:=runtime_logs/replay.csv`로 별도 지정한다.

## 2 m LiDAR 자동 반복 회피 주행 (단일 CSV, 서비스 1회)

LiDAR 점을 scan timestamp의 TF로 `odom`에 변환해 벽과 추적 장애물을 분류한다. 기준 경로는 `routes/straight_reference.csv` 하나만 프로그램 시작 시 한 번 로드하고 끝까지 같은 객체로 유지하며, 회피 경로는 CSV로 저장하지 않고 `/avoidance/selected_path` (`nav_msgs/msg/Path`)로만 메모리에서 관리한다. CSV footprint 충돌이 예상되고 LiDAR 원점에서 물리 장애물 표면까지 2.0 m 이하가 3회 확인되면 follower가 정지한다. 정지 확인 후 planner가 quintic 후보를 생성해 충돌·clearance·`±25°`(최소 회전반경 약 1.651 m)를 검사하고, 초과 후보는 클램프하지 않고 폐기한다. `PATH_READY` 후 `auto_start_avoidance:=true`(기본값)이면 `path_ready_hold_sec`(기본 0.30 s) 대기 후 서비스 호출 없이 자동으로 회피 경로를 주행한다(`/avoidance/start_selected_path`는 디버깅용으로만 남아 있다). 회피 종료 때 한 제어 주기 정지한 다음 진행 index를 역행하지 않고 CSV의 더 뒤쪽 index로 복귀하며, 통과한 장애물은 `PASSED`로 표시되어 재계획 대상에서 제외된다. 두 번째 장애물도 동일 과정이 자동 반복되고, CSV 끝에서 `GOAL_REACHED`로 최종 정지한다. `/cmd_vel`은 항상 `route_follower` 하나만 발행한다.

터미널 1 — 랜덤 장애물, follower, planner, Gazebo 및 RViz2:

```bash
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch avoidance_gazebo avoidance_planning.launch.py \
  route_file:=/home/qor/avoidance_sim_ws/routes/straight_reference.csv \
  spawn_obstacles:=true obstacle_seed:=4929145813248401951 use_rviz:=true auto_start:=true \
  planner_enabled:=true auto_start_avoidance:=true \
  replan_trigger_distance_m:=2.0 max_steering_deg:=25.0
```

터미널 2 — planner 상태:

```bash
source /opt/ros/jazzy/setup.bash
source ~/avoidance_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /avoidance/planner_status
```

최초 출발 이후 첫 번째·두 번째 장애물 회피와 CSV 복귀는 자동이며, 추가 avoidance start service는 필요하지 않다. 장애물 표면 2.0 m에서 정지하고 최대 조향각은 `25°`, 복귀 heading tolerance는 `2°`이다.

터미널 3 — 최초 출발 한 번:

```bash
source /opt/ros/jazzy/setup.bash
source ~/avoidance_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service call /avoidance/route/start std_srvs/srv/Trigger "{}"
```

전체 정지:

```bash
ros2 service call /avoidance/stop std_srvs/srv/Trigger "{}"
```

GPS 주행 계약은 `/gps_drive=2.00`과 `/gps_wheel`, 회피 주행 계약은 `/lidar_drive=1.00`과 `/lidar_wheel`이며 정지 시 `control_source=STOP`으로 두 쌍 모두 0이다. `/avoidance/control_source`는 `GPS`, `LIDAR`, `STOP` 중 하나이고 두 쌍이 섞이지 않는다. 선택/실제 회피 경로는 `/avoidance/selected_path`, `/avoidance/actual_avoidance_path`에서, 장애물별 상태(`UNSEEN/TRACKED/PLANNING/AVOIDING/PASSED`)는 `/avoidance/obstacle_status`와 `/avoidance/planner_status`의 `obstacle_statuses`에서 확인한다. `obstacle_seed:=-1`은 시스템 난수와 직전 배치 비교를 사용하고, 0 이상은 재현 테스트 전용이다. 설정은 `avoidance_planner/config/avoidance_planner.yaml`, `avoidance_route/config/route_follower.yaml`에 있다.

## S자 코스 자동 회피

S자 중심선은 양 끝에서 위치·heading·curvature가 짧은 접선 구간과 연속인 해석적 C2 곡선이다. 도로·흰선·연석은 이 중심선의 local normal offset으로 생성되며, 기준 경로 `routes/s_curve_reference.csv`는 odom 원점부터 약 0.10 m 간격으로 저장돼 있다. 공통 planner는 직선/S자 모두 CSV local Frenet `(s,d)`를 사용한다.

터미널 1:

```bash
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 daemon stop
ros2 daemon start

ros2 launch avoidance_gazebo s_curve_planning.launch.py \
  route_file:=$HOME/avoidance_sim_ws/routes/s_curve_reference.csv \
  spawn_obstacles:=true \
  obstacle_seed:=-1 \
  use_rviz:=true \
  auto_start:=false \
  planner_enabled:=true \
  auto_start_avoidance:=true \
  replan_trigger_distance_m:=2.0 \
  max_steering_deg:=25.0
```

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/avoidance_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /avoidance/planner_status
```

터미널 3 — 최초 한 번만 실행:

```bash
source /opt/ros/jazzy/setup.bash
source ~/avoidance_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service call /avoidance/route/start std_srvs/srv/Trigger "{}"
```

장애물 없는 기준 추종은 터미널 1 명령에서 `spawn_obstacles:=false`로 바꾼다. 코스 asset을 다시 생성해야 할 때만 다음 결정론적 명령을 사용하며, launch 중에는 파일을 다시 만들지 않는다.

```bash
cd ~/avoidance_sim_ws
PYTHONPATH=src/avoidance_gazebo python3 -c \
  "from avoidance_gazebo.s_curve_course import write_assets; write_assets('.')"
```
