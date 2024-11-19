from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install the package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),

        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Include resource files
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),

        # Include non-Python files from path_planner subdirectories
        (os.path.join('share', package_name, 'path_planner', 'data'), 
            [f for f in glob('drone_control/path_planner/data/**/*', recursive=True) if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'path_planner', 'objects'), 
            [f for f in glob('drone_control/path_planner/objects/**/*', recursive=True) if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'path_planner', 'stl_gen'), 
            [f for f in glob('drone_control/path_planner/stl_gen/**/*', recursive=True) if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'path_planner', 'transformation'), 
            [f for f in glob('drone_control/path_planner/transformation/**/*', recursive=True) if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bruno',
    maintainer_email='bcentanaro@gmail.com',
    description='Drone controller, gimbal stabilizer and path planner',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = drone_control.control:main',
            'gimbal_stabilizer = drone_control.gimbal_stabilizer:main',
        ],
    },
)
