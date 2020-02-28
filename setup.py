from setuptools import setup
from glob import glob

package_name = 'raspimouse_ros2_examples'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ShotaAk',
    maintainer_email='s.aoki@rt-net.jp',
    description='Sample applications for Raspberry Pi Mouse',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_control = raspimouse_ros2_examples.joystick_control:main'
        ],
    },
)
