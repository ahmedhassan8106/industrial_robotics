

import os

from launch import LaunchDescription
from launch_ros.actions import SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_share = get_package_share_directory('robot_c')
    moveit_share = get_package_share_directory('moveIt2')

    # Start Gazebo
    lab_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_share, 'launch', 'gazebo.launch.py')
        )
    )

    # Start MoveIt planning
    moveit_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'demo.launch.py')
        )
    )

        # Start MoveIt RViz
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={
            'rviz_config': os.path.join(moveit_share, 'config', 'moveit_fixed.rviz')
        }.items()
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        lab_gazebo,
        moveit_node,
        moveit_rviz,
    ])
