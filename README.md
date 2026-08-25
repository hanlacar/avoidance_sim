# avoidance_sim_ws

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim Harmonic용 직선·S자 도로 LiDAR 인지·경로 기록·2 m 정지 및 자동 반복 회피 주행 환경이다. 선택한 기준 CSV를 최초 서비스 호출 한 번으로 끝까지 주행하며, 장애물을 만날 때마다 정지·회피 경로 생성·자동 회피 주행·CSV 복귀를 서비스 재호출 없이 반복한다. 모든 ROS 실행은 CycloneDDS와 `ROS_DOMAIN_ID=12`를 사용한다.

```bash
# 터미널 1
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

colcon build --symlink-install
source install/setup.bash

ros2 launch avoidance_gazebo facility_s_curve_planning.launch.py \
  use_rviz:=true \
  auto_start:=false \
  planner_enabled:=true \
  auto_start_avoidance:=true \
  replan_trigger_distance_m:=2.0 \
  max_steering_deg:=25.0
```

```bash
# 터미널 2 — 최초 출발 한 번
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 service call /avoidance/route/start std_srvs/srv/Trigger '{}'
```

```bash
# 전체 정지
ros2 service call /avoidance/route/stop std_srvs/srv/Trigger '{}'
```
