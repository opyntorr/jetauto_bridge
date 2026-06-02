import os
from glob import glob
from setuptools import setup

package_name = 'jetauto_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='opyntorr',
    maintainer_email='a00344869@tec.mx',
    description='Odometry calibration (linear/angular) for the JetAuto; publishes /cmd_vel.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_linear = jetauto_calibration.calibrate_linear:main',
            'calibrate_angular = jetauto_calibration.calibrate_angular:main',
        ],
    },
)
