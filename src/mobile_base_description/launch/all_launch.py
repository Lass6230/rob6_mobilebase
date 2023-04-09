import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import *
import launch
import launch_ros.actions 


def generate_launch_description():
    mobile_description_dir = get_package_share_directory('mobile_base_description')
    mobile_launch = os.path.join(mobile_description_dir,'launch')
    crust_arm_moveit_config_dir = get_package_share_directory('crust_arm_moveit_config')
    crust_launch = os.path.join(crust_arm_moveit_config_dir,'launch')
    opencv_detector_dir = get_package_share_directory('opencv_detector')
    opencv_launch = os.path.join(opencv_detector_dir,'launch')

    lidar_dir = get_package_share_directory('sick_safetyscanners2')
    lidar_launch = os.path.join(lidar_dir,'launch')


    # bringup_cmd_group = GroupAction([
    headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mobile_launch,'headless_navigation_launch.py')),
            )
    
    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(crust_launch,'real_base_gripper.launch.py')),
        launch_arguments={
            'headless': 'true',
        }.items()
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(opencv_detector_dir,'rs_launch.py')),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_launch,'sick_safetyscanners2_launch.py')),

    )



    main_program_node = launch_ros.actions.Node(
       package= 'mobile_base_description',
       executable='main_program',
       name='main_program',
    )

    joy_control_node = launch_ros.actions.Node(
        package= 'mobile_base_description',
        executable='joy_listner',
        name='joy_listner',
    )


    return LaunchDescription([
        

        RegisterEventHandler(
            OnExecutionComplete(
                target_action=lidar,
                on_completion=[
                    headless,
                ]
            )
        ),

        RegisterEventHandler(
            OnExecutionComplete(
                target_action=headless,
                on_completion=[
                    realsense,
                ]
            )
        ),

        RegisterEventHandler(
            OnExecutionComplete(
                target_action=realsense,
                on_completion=[
                    gripper,
                ]
            )
        ),

        RegisterEventHandler(
            OnExecutionComplete(
                target_action=gripper,
                on_completion=[
                    joy_control_node,
                ]
            )
        ),

        # RegisterEventHandler(
        #     OnExecutionComplete(
        #         target_action=joy_control_node,
        #         on_start=[
        #             main_program_node,
        #         ]
        #     )
        # ),

        
        lidar,


    ])