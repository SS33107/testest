import os
# import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package

 
def generate_launch_description():
 
  # Set the path to the Gazebo ROS package
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_tm5_900_pkg_share = FindPackageShare(package='tm_moveit_config_tm5-900').find('tm_moveit_config_tm5-900')  
  pkg_tm_description_share = FindPackageShare(package='tm_description').find('tm_description')


  # Set the path to the world file
  world_file_name = 'arm_with_table.world'
  # world_path = os.path.join(pkg_tm5_900_pkg_share, 'worlds', world_file_name)
  world_path = '/home/ubuntu20auo/Documents/TM_ros2/src/tmr_ros2/tm_moveit_config_tm5-900/worlds/arm_with_table.world'

  # Set the path to the urdf file
  # urdf_file_name = 'tm5-900.urdf.xacro'
  # urdf_model_path = os.path.join(pkg_tm_description_share, 'xacro', urdf_file_name)
  urdf_model_path = '/home/ubuntu20auo/Documents/TM_ros2/src/tmr_ros2/tm_description/urdf/tm5-900-nominal.urdf'
   
  # Set the path to the rviz2 file
  rviz2_file_name = 'tm5_gazebo.rviz'
  rviz_config_path = os.path.join(pkg_tm5_900_pkg_share, 'rviz', rviz2_file_name)


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
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  urdf_model = LaunchConfiguration('urdf_model')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

  robot_x = LaunchConfiguration('0')
  robot_y = LaunchConfiguration('0')
  robot_z = LaunchConfiguration('0')
  robot_rx = LaunchConfiguration('0')
  robot_ry = LaunchConfiguration('0')
  robot_rz = LaunchConfiguration('0')
 

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
  
  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name          ='urdf_model', 
    default_value =urdf_model_path, 
    description   ='Absolute path to robot urdf file')
 
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name          ='use_robot_state_pub',
    default_value ='True',
    description   ='Whether to start the robot state publisher')


  declare_use_rviz_cmd = DeclareLaunchArgument(
    name          ='use_rviz',
    default_value ='True',
    description   ='Whether to start RVIZ')

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name          ='rviz_config_file',
    default_value =rviz2_file_name,
    description   ='Full path to the RVIZ config file to use')
 
  
 
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
                    '-topic', '/robot_description',
                    '-x', '0', '-y', '0', '-z', '0',
                    '-R', '0', '-P', '0', '-Y', '0',
                    # '-J','shoulder_1_joint 0.5', '-J','shoulder_2_joint 0',
                    # '-J','elbow_joint 0', '-J','wrist_1_joint 0',
                    # '-J','wrist_2_joint 0', '-J','wrist_3_joint 0'
                    ],
    output     ='screen')

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
  start_robot_state_publisher_cmd = Node(
    package    ='robot_state_publisher',
    executable ='robot_state_publisher',
    name       ='robot_state_publisher',
    output     ='both',
    parameters =[{'robot_description': Command(['xacro ', urdf_model_path])}])
 
  # Publish the joint states of the robot
  start_joint_state_publisher_cmd = Node(
    package    ='joint_state_publisher',
    executable ='joint_state_publisher',
    name       ='joint_state_publisher',
    condition  =UnlessCondition(gui))

  # Publish the joint states of the robot
  start_joint_state_gui_publisher_cmd = Node(
    package    ='joint_state_publisher_gui',
    executable ='joint_state_publisher_gui',
    name       ='joint_state_publisher_gui',
    output     ='screen')


  # Launch RViz
  start_rviz_cmd = Node(
    package    ='rviz2',
    executable ='rviz2',
    name       ='rviz2',
    output     ='screen',
    arguments  =['-d', rviz_config_file])


  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_world_cmd)
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)

  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd) 
  ld.add_action(declare_use_joint_state_publisher_cmd)
  
  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  # ld.add_action(spawn_entity_cmd)
  # ld.add_action(start_robot_state_publisher_cmd)
  # ld.add_action(start_joint_state_gui_publisher_cmd)
  # ld.add_action(start_rviz_cmd)
  
  return ld