from setuptools import setup
import os
from glob import glob

package_name = 'example_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        "example_package",
        "example_package.bt_nodes",  # Explicitly register bt_nodes module
        "example_package.server",
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_simulation.launch.py']),
        (os.path.join('lib', package_name), []),  # Mark this as a destination for executables
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Example package for robot simulation with ROS2 and ros_bt_py',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_server = example_package.server.action_server:main',
            'battery_simulator = example_package.battery_simulator:main'
        ],
    },
    scripts=[],
    python_requires='>=3.6',
)