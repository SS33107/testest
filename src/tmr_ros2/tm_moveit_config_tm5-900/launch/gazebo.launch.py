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



def load_file(package_name, file_path):
  package_path = get_package_share_directory(package_name)
  absolute_file_path = os.path.join(package_path, file_path)
  try:
    with open(absolute_file_path, 'r') as file:
      return file.read()
  except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
    return None

def generate_launch_description():
 
  # Set the path to the Gazebo ROS package
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_tm5_900_pkg_share = FindPackageShare(package='tm_moveit_config_tm5-900').find('tm_moveit_config_tm5-900')  
  pkg_tm_description_share = FindPackageShare(package='tm_description').find('tm_description')


  # Set the path to the world file
  world_file_name = 'arm_with_table.world'
  world_path = os.path.join(pkg_tm5_900_pkg_share, 'worlds', world_file_name)
  # world_path = '/home/ubuntu20auo/Documents/TM_ros2/src/tmr_ros2/tm_moveit_config_tm5-900/worlds/arm_with_table.world'

  
  # Set the path to the rviz2 file
  rviz2_file_name = 'tm5_gazebo.rviz'
  rviz_config_path = os.path.join(pkg_tm5_900_pkg_share, 'rviz', rviz2_file_name)

  # Set the path to the urdf file
  robot_description_config = xacro.process_file(os.path.join(get_package_share_directory("tm_description"),"xacro","tm5-900-ros2.urdf.xacro"))  
  # urdf_model_path = get_package_share_directory('tm_description') + '/xacro/' + 'tm5-900-ros2.urdf.xacro'
  # robot_description_config = xacro.process_file(urdf_model_path)
  # robot_description_config = xacro.process_file("/home/ubuntu20auo/Documents/robotic_arm_environment-main/src/my_doosan_pkg/description/xacro/a0912.urdf.xacro")  
  xacro_file = os.path.join(get_package_share_directory("tm_description"),"xacro","tm5-900-ros2.urdf.xacro")
  # params  = {"robot_description": Command(['xacro', ' ', xacro_file])}
  
  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  namespace = LaunchConfiguration('namespace')
  use_namespace = LaunchConfiguration('use_namespace')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  use_rviz = LaunchConfiguration('use_rviz')
  world = LaunchConfiguration('world')
  gui = LaunchConfiguration('gui')
  # rviz_config_file = LaunchConfiguration('rviz_config_file')
  urdf_model = LaunchConfiguration('urdf_model')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

  spawn_x_val = '0.0'
  spawn_y_val = '0.0'
  spawn_z_val = '0.0'
  spawn_yaw_val = '0.00'

  params = {"robot_description": robot_description_config.toxml()}
  robot_description_semantic_config = load_file('tm_moveit_config_tm5-900', 'config/tm5-900.srdf')
  robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}
  aa = { Command(['xacro ', xacro_file])}

  declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name          ='gui',
    default_value ='True',
    description   ='Flag to enable joint_state_publisher_gui')

  declare_namespace_cmd = DeclareLaunchArgument(
    name          ='namespace',
    default_value ='',
    description   ='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name          ='use_namespace',
    default_value ='false',
    description   ='Whether to apply a namespace to the navigation stack')

  declare_simulator_cmd = DeclareLaunchArgument(
    name          ='headless',
    default_value ='False',
    description   ='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name          ='use_sim_time',
    default_value ='true',
    description   ='Use simulation (Gazebo) clock if true')

 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name          ='use_simulator',
    default_value ='True',
    description   ='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name          ='world',
    default_value = world_path,
    description   ='Full path to the world model file to load')
 
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name          ='use_robot_state_pub',
    default_value ='True',
    description   ='Whether to start the robot state publisher')

 
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

  # Launch the robot
  spawn_entity_cmd = Node(
    package    ='gazebo_ros', 
    executable ='spawn_entity.py',
    name       = 'spawn_entity',
    arguments  =['-entity', 'tm5-900', 
                    '-topic', 'robot_description',
                    '-x', spawn_x_val, '-y', spawn_y_val, '-z', spawn_z_val,
                    # '-R', '0', '-P', '0', '-Y', '0'
                    # '-J','shoulder_1_joint 0.5', '-J','shoulder_2_joint 0',
                    # '-J','elbow_joint 0', '-J','wrist_1_joint 0',
                    # '-J','wrist_2_joint 0', '-J','wrist_3_joint 0'
                    ],
    output     ='both')

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
  start_robot_state_publisher_cmd = Node(
    package    ='robot_state_publisher',
    executable ='robot_state_publisher',
    name       ='robot_state_publisher',
    output     ='both',
    # parameters = [{'robot_description': params}])
    parameters = [params])
    # parameters=[{ Command(['xacro ', xacro_file])}])

  # Publish the joint states of the robot
  start_joint_state_publisher_cmd = Node(
    package    ='joint_state_publisher',
    executable ='joint_state_publisher',
    name       ='joint_state_publisher',
    output     ='both')

  # Publish the joint states of the robot
  start_joint_state_gui_publisher_cmd = Node(
    package    ='joint_state_publisher_gui',
    executable ='joint_state_publisher_gui',
    name       ='joint_state_publisher_gui',
    output     ='both')

  
  
  load_joint_trajectory_controller = ExecuteProcess( 
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_velocity_controller'], 
                    output='both')

  load_joint_state_broadcaster = ExecuteProcess( 
                  cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'], 
                  output='both')

  # Launch RViz
  start_rviz_cmd = Node(
    package    ='rviz2',
    executable ='rviz2',
    name       ='rviz2',
    output     ='both',
    arguments  =['-d', rviz_config_path],
    parameters = [params,robot_description_semantic])


  # controllers_file = os.path.join(pkg_tm5_900_pkg_share,'config','arm_controller_vel.yaml' )

  # controller_manager = Node(
  #   package="controller_manager",
  #   executable="ros2_control_node",
  #   parameters=[params,controllers_file],
  #   output='screen'
  # )



  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_world_cmd)
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_simulator_cmd)
  # ld.add_action(declare_rviz_config_file_cmd)
  # ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd) 
  ld.add_action(declare_use_joint_state_publisher_cmd)
  

  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)

  ld.add_action(load_joint_trajectory_controller)
  ld.add_action(load_joint_state_broadcaster)

  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_joint_state_publisher_cmd)
  # ld.add_action(start_joint_state_gui_publisher_cmd)

  ld.add_action(spawn_entity_cmd)
  # ld.add_action(controller_manager)
  # ld.add_action(start_rviz_cmd)
  
  return ld
