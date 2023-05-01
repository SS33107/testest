import os
import sys
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    args = ['robot_ip:=192.168.10.2']
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    # Component yaml files are grouped in separate namespaces
    # Use URDF file: tm5-900-nominal.urdf to do moveit demo
    # robot_description_config = load_file('tm_description', 'urdf/tm5-900-nominal.urdf')
    # robot_description = {'robot_description' : robot_description_config}
    # Use Xacro file: tm5-900.urdf.xacro to do moveit demo
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("tm_description"),
            "xacro",
            "tm5-900.urdf.xacro",
        )
    )                                                    
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file('tm_moveit_config_tm5-900', 'config/tm5-900.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}
    
    # RViz
    rviz_config_file = get_package_share_directory('tm_driver') + "/rviz/bringup.rviz"
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic]
        )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # joint driver
    tm_driver_node = Node(
        package='tm_driver',
        executable='tm_driver',
        #name='tm_driver',
        output='screen',
        arguments=args
    )
    

    return LaunchDescription([ tm_driver_node, static_tf, robot_state_publisher, rviz_node ])
