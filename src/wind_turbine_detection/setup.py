from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wind_turbine_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'wind_turbine_detection.image_subscriber',
        'wind_turbine_detection.processes'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'resource'),
         [f for f in glob(os.path.join('resource', package_name, '*')) if os.path.isfile(f)]),
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
            'processes = wind_turbine_detection.processes:main',
            'image_subscriber = wind_turbine_detection.image_subscriber:main',
        ],
    },
)
