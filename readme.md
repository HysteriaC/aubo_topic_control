# AUBO ROS Topic Control
**Maintainer: luxin@aubo-robotics.cn**
### Usage with real robot
```
source devel/setup.bash
roslaunch aubo_robot_driver rviz_robot.launch ip:=192.168.63.128

other terminal run:
source devel/setup.bash
rosrun aubo_robot_driver test.py
or topic control
rostopic pub --once /robot_commands std_msgs/String "data: 'MOVJ 40 40 40 40 40 40'"

```
