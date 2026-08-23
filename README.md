# avoidance_sim_ws

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim Harmonic용 직선도로 LiDAR·모범경로 기록 환경이다. 자동 회피나 자동 정지/조향은 포함하지 않는다.

## 의존성과 빌드

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-xacro ros-jazzy-sensor-msgs-py python3-yaml
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 터미널별 실행 순서

터미널 1 — 장애물 없는 Gazebo, LiDAR 처리, RViz2 및 실제 odometry 기록:

```bash
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch avoidance_gazebo straight_avoidance.launch.py \
  spawn_obstacles:=false use_rviz:=true record_route:=true \
  route_file:=/home/qor/avoidance_sim_ws/routes/straight_reference.csv
```

터미널 2 — 수동 주행(`w/s`: 전후진, `a/d`: 좌우, `space`: 정지, `q`: 종료):

```bash
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run avoidance_route manual_teleop
```

기존 파일을 덮어쓰지 않으며 같은 이름이 있으면 timestamp가 붙은 새 CSV를 만든다. 기록과 시뮬레이션은 각 터미널에서 `Ctrl+C`로 정상 종료한다.

장애물 검출 테스트:

```bash
ros2 launch avoidance_gazebo straight_avoidance.launch.py spawn_obstacles:=true use_rviz:=true
```

RViz만 다시 실행하려면:

```bash
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
