import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


# True  = ROS 2 Control (use this with MoveIt2 / RViz)
# False = Original Gazebo JointPositionControllers + sliders

USE_ROS2_CONTROL = True


def generate_launch_description():



    share = Path(get_package_share_directory('robot_c'))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))

    world = share / 'worlds' / 'milestone.sdf'
    bridge_config = share / 'config' / 'bridge.yaml'
    controller_config = share / 'config' / 'joint_controller.yaml'


    description = xacro.process_file(
        str(share / 'urdf' / 'simulation.urdf.xacro'),
        mappings={
            'gazebo': 'true',
            'ros2_control': 'true' if USE_ROS2_CONTROL else 'false'
        }
    ).toxml()


    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=(
            str(share.parent)
            + os.pathsep
            + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        )
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gz_share / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r "' + str(world) + '"'
        }.items()
    )


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': description,
            'use_sim_time': True
        }],
        output='screen'
    )


    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='create_robot',
        arguments=[
            '-world', 'milestone',
            '-name', 'simulation',
            '-topic', 'robot_description'
        ],
        output='screen'
    )


    if USE_ROS2_CONTROL:
        # ros2_control publishes /joint_states itself.
        # Bridge only the simulation clock.
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen'
        )
    else:
        # Original bridge for native Gazebo controllers
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{
                'config_file': str(bridge_config)
            }],
            output='screen'
        )


    actions = [
        resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ]



    if not USE_ROS2_CONTROL:

        slider_controller = Node(
            package='robot_c',
            executable='slider_to_gazebo_c',
            name='slider_to_gazebo_c',
            output='screen'
        )

        rviz_sliders = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{
                'robot_description': description,
                'rate': 20,
                'source_list': ['/gazebo_slider_targets']
            }],
            remappings=[
                ('/joint_states', '/slider_joint_targets')
            ],
            output='screen'
        )

        gazebo_sliders = Node(
            package='robot_c',
            executable='gazebo_slider_panel_c',
            name='gazebo_slider_panel_c',
            output='screen'
        )

        actions += [
            slider_controller,
            rviz_sliders,
            gazebo_sliders,
        ]



    if USE_ROS2_CONTROL:

        # The Gazebo plugin creates controller_manager.
        # This spawner loads and activates all controllers
        # after the simulation starts.

        controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='robot_controller_spawner',
            arguments=[
                'joint_state_broadcaster',
                'arm_group_controller',
                'hand_group_controller',
                '--controller-manager',
                '/controller_manager',
                '--param-file',
                str(controller_config),
                '--controller-manager-timeout',
                '120',
                '--service-call-timeout',
                '120'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

        actions.append(
            TimerAction(
                period=5.0,
                actions=[controller_spawner]
            )
        )


    return LaunchDescription(actions)
