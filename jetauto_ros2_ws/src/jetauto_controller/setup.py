import os
from glob import glob
from setuptools import setup

package_name = 'jetauto_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='opyntorr',
    maintainer_email='a00344869@tec.mx',
    description='JetAuto mecanum base driver for ROS2 (I2C motors + MPU6050 IMU + odom).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_node = jetauto_controller.chassis_node:main',
            'imu_node = jetauto_controller.imu_node:main',
            'battery_node = jetauto_controller.battery_node:main',
        ],
    },
)
