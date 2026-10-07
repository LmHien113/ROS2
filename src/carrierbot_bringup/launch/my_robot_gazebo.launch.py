import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get ROS distro
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    
    # Set is_ignition based on distro
    is_ignition = 'True' if ros_distro == "humble" else 'False'

    # Package paths
    my_robot_description_dir = get_package_share_directory("carrierbot_description")
    my_robot_bringup_dir = get_package_share_directory("carrierbot_bringup")
    robot_controller_dir = get_package_share_directory("carrierbot_controller")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    
    # File paths
    urdf_path = os.path.join(my_robot_description_dir, "urdf", "my_robot.urdf.xacro")
    rviz_config_path = os.path.join(my_robot_description_dir, "rviz", "urdf_config.rviz")
    gazebo_config_path = os.path.join(my_robot_bringup_dir, "config", "gazebo_bridge.yaml")
    world_path = os.path.join(my_robot_bringup_dir, "worlds", "test_world.sdf")
    ros2_control_config_path = os.path.join(robot_controller_dir, "config", "ros2_control.yaml")
    # World selection: comment/uncomment to switch
    use_empty_world = True  # Set to True for empty world, False for test_world.sdf
    
    if use_empty_world:
        gz_world_args = 'empty.sdf -r'
    else:
        gz_world_args = f'{world_path} -r'
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path, ' is_ignition:=', is_ignition])
        }]
    )
    
    # Gazebo Sim
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            'gz_args': gz_world_args
        }.items()
    )
    
    # Spawn Robot
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"]
    )
    
    # ROS-Gazebo Bridge
    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[gazebo_config_path]
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen"
    )
    
    # Include Controller Launch File (delayed to ensure robot is spawned first)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_controller_dir, "launch", "controller.launch.py")
        )
    )
    
    # Delay controller spawning by 5 seconds to ensure Gazebo and robot are ready
    delayed_controller_launch = TimerAction(
        period=5.0,
        actions=[controller_launch]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_sim,
        spawn_robot_node,
        ros_gz_bridge_node,
        rviz_node,
        delayed_controller_launch,
    ])
