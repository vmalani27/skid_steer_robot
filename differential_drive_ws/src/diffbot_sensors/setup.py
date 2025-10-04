from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diffbot_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rovasura',
    maintainer_email='vanshmalani27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ultrasonic_sensor = diffbot_sensors.ultrasonic_sensor:main',
            'multi_ultrasonic = diffbot_sensors.multi_ultrasonic:main',
            'ultrasonic_test = diffbot_sensors.ultrasonic_test:main',
            'sensor_aggregator = diffbot_sensors.sensor_aggregator:main',
        ],
    },
)
