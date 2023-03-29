
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import yaml
import os

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
    
    kinematics_yaml = load_yaml(
        "crust_arm_moveit_config", "config/kinematics_gripper.yaml"
    )

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("crust_arm_moveit_config"),
            "config",
            "crust_real_gripper.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "crust_arm_moveit_config", "config/crust_arm_mobile_gripper.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="server",
        package="crust_arm_moveit_config",
        executable="server",
        output="screen",
        parameters=[
           # moveit_config.robot_description,
            #moveit_config.robot_description_semantic,
            kinematics_yaml,
            robot_description,
            robot_description_semantic,
        ],
    )

    return LaunchDescription([move_group_demo])