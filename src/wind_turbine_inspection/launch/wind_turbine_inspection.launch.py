from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('wind_turbine_inspection')
    detection_launch_path = os.path.join(
        get_package_share_directory('wind_turbine_detection'),
        'wind_turbine_detection.launch.py'
    )
    control_launch_path = os.path.join(
        get_package_share_directory('drone_control'),
        'drone_control.launch.py'
    )
    return LaunchDescription([
        Node(
            package='wind_turbine_inspection',
            namespace='wind_turbine_inspection',
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='wind_turbine_inspection',
            namespace='wind_turbine_inspection',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        # Node(
        #     package='wind_turbine_inspection',
        #     namespace='wind_turbine_inspection',
        #     executable='mission_state_handler',
        #     name='mission_state_handler',
        #     prefix='gnome-terminal --',
        #     output='screen'
        # ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        ),
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource(detection_launch_path)
        )
        ,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch_path)
        )
    ])
