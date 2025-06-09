from setuptools import find_packages, setup

package_name = 'ros2_hoverrobot_comms'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['resource', 'resource.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
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
            'hoverrobot_comms_node = ros2_hoverrobot_comms.hoverrobot_comms_node:main'
        ],
    },
)
