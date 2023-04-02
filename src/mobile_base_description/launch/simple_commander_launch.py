# Copyright (c) 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    #warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    #python_commander_dir = get_package_share_directory('nav2_simple_commander')
    mobile_base_dir = get_package_share_directory('mobile_base_description')

    map_yaml_file = os.path.join(mobile_base_dir, 'maps/hall2', 'map3.yaml')
    #world = os.path.join(python_commander_dir, 'warehouse.world')

    # # start the simulation
    # start_gazebo_server_cmd = ExecuteProcess(
    #     cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', world],
    #     cwd=[warehouse_dir], output='screen')

    # start_gazebo_client_cmd = ExecuteProcess(
    #     cmd=['gzclient'],
    #     cwd=[warehouse_dir], output='screen')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    'use_robot_state_pub',
    default_value='False',
    description='Whether to start the robot state publisher')
   
    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("crust_arm_moveit_config"),
            "config",
            "base.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

  
    

    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False'}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_yaml_file}.items())

    # start the demo autonomy task
    demo_cmd = Node(
        package='nav2_simple_commander',
        executable='example_nav_to_pose',
        emulate_tty=True,
        output='screen')
    
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    odomZOH = Node(
        package='mobile_base_description',
        executable='odom_rebroadcaster',
        name='odom_rebroadcaster',
        output='screen'
    )


    ld = LaunchDescription()
    #ld.add_action(start_gazebo_server_cmd)
    #ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(demo_cmd)
    ld.add_action(odomZOH)
    return ld
