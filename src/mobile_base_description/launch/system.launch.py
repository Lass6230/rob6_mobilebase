import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro
import launch_ros
from nav2_common.launch import HasNodeParams
import launch




def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='mobile_base_description').find('mobile_base_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_params_file = os.path.join(get_package_share_directory("mobile_base_description"),
                                       'config', 'mapper_params_online_async.yaml')

    # Command-line arguments
    
    headless = DeclareLaunchArgument(
        "headless", default_value="false", description="run headless: default false"
    )
    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("crust_arm_moveit_config"),
            "config",
            "base.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

   

    # RViz
    headless_mode = LaunchConfiguration("headless")
    rviz_base = os.path.join(get_package_share_directory("crust_arm_moveit_config"), "launch")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_empty_config = os.path.join(rviz_base, "moveit_empty.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_empty_config],
        parameters=[
            robot_description,
            
        ],
        
        condition=UnlessCondition(headless_mode),
    )
    
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Publish arbitrary joint angles
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        #condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    has_node_params = HasNodeParams(source_file=params_file,
                                    node_name='slam_toolbox')

    actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                           ' else "', default_params_file, '"'])

    log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                    ' does not contain slam_toolbox parameters. Using default: ',
                                    default_params_file],
                               condition=UnlessCondition(has_node_params))

    start_async_slam_toolbox_node = Node(
        parameters=[
          actual_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )
    
    odomZOH = Node(
        package='mobile_base_description',
        executable='odom_rebroadcaster',
        name='odom_rebroadcaster',
        output='screen'
    )

    

    return LaunchDescription(
        [   
            headless,
            #rviz_node,
            robot_state_publisher,
            declare_use_sim_time_argument,
            declare_params_file_cmd,
            log_param_change,
            #start_async_slam_toolbox_node,
            odomZOH
            
        ]   
        
    )
