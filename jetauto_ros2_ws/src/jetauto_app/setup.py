import os
from glob import glob
from setuptools import setup

package_name = 'jetauto_app'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='opyntorr',
    maintainer_email='a00344869@tec.mx',
    description='JetAuto vision/lidar demo apps (ROS2 port).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_app = jetauto_app.lidar_app:main',
            'line_following = jetauto_app.line_following:main',
            'object_tracking = jetauto_app.object_tracking:main',
            'patrol = jetauto_app.patrol:main',
            'ar_app = jetauto_app.ar_app:main',
        ],
    },
)
