#!/bin/bash

sh /home/user/brdk_agv/ros2_ws/src/motor_control_exos/Run2.sh &

ros2 launch sick_safetyscanners2 sick_safetyscanners2_launch.py &

ros2 launch mobile_base_description headless_navigation_launch.py slam:=true &

ros2 launch crust_arm_moveit_config real_base_gripper.launch.py &

ros2 run mobile_base_description main_program &



wait

