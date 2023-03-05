# ArmACup

### AGAIN AND AGAIN
- rosdep install -i --from-path src --rosdistro foxy -y
- colcon build
- . install/setup.bash

### Source ROS 2 environment
- source /opt/ros/foxy/setup.bash

### Turtlesim
- sudo apt install ros-foxy-turtlesim
- ros2 run turtlesim turtlesim_node
- ros2 run turtlesim turtle_teleop_key
- ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel

- ros2 node list
- ros2 node info "node name"

### RQT
- rqt
- rqt_graph
- ros2 run rqt_console rqt_console

### Create a workspace
- mkdir -p ~/ros2_ws/src
- cd ~/ros2_ws
- colcon build [--symlink-install]
- rosdep install -i --from-path src --rosdistro foxy -y
- . install/setup.bash

### Create a package
- ros2 pkg create --build-type ament_python <package_name>


### inPackage
#### Python
- setup.py
```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
```
#### CPP
- CMakeList.txt
```
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

