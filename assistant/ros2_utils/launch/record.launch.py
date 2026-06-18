import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, ThisLaunchFileDir

def generate_launch_description():

    dir = DeclareLaunchArgument('dir', default_value='./log/ros_bags', description='')

    return launch.LaunchDescription([
        dir,
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', LaunchConfiguration('dir')],
            output='screen'
        )
    ])
