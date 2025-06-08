# ros2_hoverrobot_comms/setup.py
from setuptools import setup

package_name = 'ros2_hoverrobot_comms'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['resource/' + package_name])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pato Garcés',
    maintainer_email='patricio.garces@outlook.com',
    description='Comms module for HoverRobot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hoverrobot_comms_node = ros2_hoverrobot_comms.main:main'
        ],
    },
)
