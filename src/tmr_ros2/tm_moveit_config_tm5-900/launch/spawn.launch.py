# Author: Addison Sears-Collins
# Date: September 19, 2021
# Description: Load a world file into Gazebo.
# https://automaticaddison.com
 
import os
import sys
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package
 
def generate_launch_description():


    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory("tm_description"),"xacro","tm5-900-ros2.urdf.xacro")) 
    params = {"robot_description": robot_description_config.toxml()}

    spawn_entity_cmd = Node(
    package    ='gazebo_ros', 
    executable ='spawn_entity.py',
    name       = 'spawn_entity',
    arguments  =['-entity', 'tm5-900', 
                    '-topic', 'robot_description',
                    # '-x', spawn_x_val, '-y', spawn_y_val, '-z', spawn_z_val,
                    # '-R', '0', '-P', '0', '-Y', '0'
                    # '-J','shoulder_1_joint 0.5', '-J','shoulder_2_joint 0',
                    # '-J','elbow_joint 0', '-J','wrist_1_joint 0',
                    # '-J','wrist_2_joint 0', '-J','wrist_3_joint 0'
                    ],
    output     ='screen')

   return LaunchDescription([ spawn_entity_cmd ])