from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
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
        'launch/drone_control.launch.py'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_arg', 
            default_value='0',  # Valor por defecto
            description='Argument for mission state handler'
        ),
        Node(
            package='wind_turbine_inspection',
            namespace='wind_turbine_inspection',
            executable='visualizer',
            name='visualizer',
            prefix='gnome-terminal --'
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
            package='wind_turbine_inspection',
            executable='mission_state_handler',
            name='mission_state_handler',
            # prefix='gnome-terminal --',
            # output='screen',
            parameters=[{'mission_param': LaunchConfiguration('mission_arg')}]
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]],
            prefix='gnome-terminal --'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(detection_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch_path)
        )
    ])
