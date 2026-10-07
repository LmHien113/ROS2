import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_bringup"),
            "launch",
            "my_robot_gazebo.launch.py"
        )
    )

    # controller = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("robot_controller"),
    #         "launch",
    #         "controller.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_simple_controller": "False",
    #         "use_python": "False"
    #     }.items()
    # )

    return LaunchDescription([
        gazebo,
        # controller,
    ])