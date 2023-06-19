#!/bin/bash

source /opt/ros/galactic/setup.bash
source /home/user/rob6_mobilebase/install/setup.bash
source /home/user/ros2_ws/install/setup.bash
sleep 15

sh /home/user/brdkAGV_linux/ros_ws/src/motor_control_exos/run3.sh &

sleep 10

ros2 launch sick_safetyscanners2 sick_safetyscanners2_launch.py &

ros2 launch mobile_base_description headless_navigation_launch.py &

ros2 launch opencv_detector rs_launch.py &

ros2 launch crust_arm_moveit_config real_base_gripper.launch.py headless:=true &

# sleep 35
# ros2 run mobile_base_description main_program &



wait

