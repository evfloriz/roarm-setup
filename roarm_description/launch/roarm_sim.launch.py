# Author: Williams
# Date: Nov 1, 2022
# Description: Launch a RoArm-M2-S URDF file using Rviz.
# https://waveshare.com
 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
def generate_launch_description():
 
  # Set the path to this package.
  pkg_share = FindPackageShare(package='roarm_description').find('roarm_description')
 
  # Set the path to the RViz configuration settings
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/view_description.rviz')
 
  # Set the path to the URDF file
  default_urdf_model_path = os.path.join(pkg_share, 'urdf/robot.urdf.xacro')
 
  # Launch configuration variables specific to simulation
  gui = LaunchConfiguration('gui')
  urdf_model = LaunchConfiguration('urdf_model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
 
  # Declare the launch arguments  
  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')
     
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')
   
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
 
  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
 
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': Command(['xacro ', urdf_model])}],
    arguments=[default_urdf_model_path])
 
  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])
  

  # Launch gz
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      FindPackageShare('ros_gz_sim').find('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )]), launch_arguments={'gz_args': '-r empty.sdf'}.items()
  )

  spawn_entity = Node(package='ros_gz_sim',
                      executable='create',
                      arguments=['-topic', 'robot_description',
                                  '-name', 'roarm'],
                                  #'-z', '0.0850'],
                      output='screen')

  # Launch bridge
  bridge_params = os.path.join(pkg_share,'config','bridge.yaml')
  bridge = Node(
      package='ros_gz_bridge',
      executable='parameter_bridge',
      arguments=['--ros-args', '-p', 'config_file:=' + bridge_params]
  )
  
  # Launch controllers
  pos_cont_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["pos_cont"]
  )

  joint_broad_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["joint_broad"]
  )
   
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
 
  # Start rsp
  ld.add_action(start_robot_state_publisher_cmd)

  # Start rviz
  ld.add_action(start_rviz_cmd)
  
  # Start gz
  ld.add_action(gazebo)
  ld.add_action(spawn_entity)
  ld.add_action(bridge)

  # Start controllers
  ld.add_action(pos_cont_spawner)
  ld.add_action(joint_broad_spawner)
 
  return ld
