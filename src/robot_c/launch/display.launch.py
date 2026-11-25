from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Find robot_c package
    share = Path(get_package_share_directory('robot_c'))

    # Read and process robot URDF
    robot_description = xacro.process_file(
        str(share / 'urdf' / 'simulation.urdf.xacro')
    ).toxml()

    return LaunchDescription([

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }],
            output='screen'
        ),

        # Original seven-joint slider GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                str(share / 'config' / 'arm.rviz')
            ],
            output='screen'
        ),

    ])
