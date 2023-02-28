import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
import xacro
import yaml
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
    #default_model_path = os.path.join(pkg_share, 'urdf/mobile_base.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config/urdf_config.rviz')
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("mobile_base_description"),
            "urdf",
            "mobile_base_gazebo.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        #parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'mobile_base', '-topic', 'robot_description'],
        output='screen'
    )
    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
       # launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
        #                                    description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        rviz_node
    ])