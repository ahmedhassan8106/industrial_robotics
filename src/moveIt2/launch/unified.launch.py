from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def _setup(context):
    mode = LaunchConfiguration('control_mode').perform(context)
    manual = mode == 'manual'
    robot = Path(get_package_share_directory('robot_c'))
    moveit = Path(get_package_share_directory('moveIt2'))

    def include(path, **args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)),
            launch_arguments=args.items(),
        )

    return [
        SetParameter(name='use_sim_time', value=True),
        LogInfo(msg='Control mode: ' + mode),
        include(robot / 'launch/gazebo.launch.py',
                use_ros2_control='true',
                manual_sliders='true' if manual else 'false'),
        include(moveit / 'launch/demo.launch.py',
                allow_trajectory_execution='false' if manual else 'true'),
        include(moveit / 'launch/moveit_rviz.launch.py',
                rviz_config=str(moveit / 'config/moveit_fixed.rviz')),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('control_mode', default_value='manual',
                              choices=['manual', 'moveit']),
        OpaqueFunction(function=_setup),
    ])
