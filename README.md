# avoidance_sim_ws

ROS 2 Jazzy + Gazebo Sim straight obstacle course.

- Road: 15.0 m x 1.95 m
- Curbs: 0.10 m high
- Vehicle: 1.30 m x 0.78 m, wheel diameter 0.30 m, wheelbase 0.77 m
- Vehicle spawn centre: start line x=1.00 m
- Obstacles: 1.30 m x 0.78 m, x=3.0 / 7.0 m, opposite LEFT/RIGHT sides
- Obstacles are embedded in a generated world before Gazebo starts; they do not depend on delayed create calls.

Run:

```bash
cd ~/avoidance_sim_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch avoidance_gazebo straight_avoidance.launch.py
```
