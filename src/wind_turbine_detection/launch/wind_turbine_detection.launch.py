from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('wind_turbine_detection')
    return LaunchDescription([
        Node(
            package='wind_turbine_detection',
            executable='image_subscriber',
            name='image_subscriber',
            # prefix='gnome-terminal --',
            # output='screen'
        ),
        Node(
            package='wind_turbine_detection',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --',
            output='screen'
        ),
    ])
