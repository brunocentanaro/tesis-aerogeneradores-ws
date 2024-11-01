from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wind_turbine_inspection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.states'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
         ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name),
         glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bruno',
    maintainer_email='bcentanaro@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processes = wind_turbine_inspection.processes:main',
            'visualizer = wind_turbine_inspection.visualizer:main',
            'controlV2 = wind_turbine_inspection.controlV2:main',
            'mission_state_handler = wind_turbine_inspection.mission_state_handler:main',
            'testing_helper = wind_turbine_inspection.testing_helper:main',
            'camera_handler = wind_turbine_inspection.camera_handler:main',
        ],
    },
)
