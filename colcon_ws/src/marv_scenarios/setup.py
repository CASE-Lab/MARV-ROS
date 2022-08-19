from setuptools import setup
import os
from glob import glob

package_name = 'marv_scenarios'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['library/timer.py']),
        ('lib/' + package_name, ['library/scenario.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.csv')),
        ('lib/' + package_name, ['library/waypoint_algorithm.py']),
        ('lib/' + package_name, ['library/thrust_controller.py']),
        ('lib/' + package_name, ['library/heading_controller.py']),
        ('lib/' + package_name, ['library/control_helper.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viktor',
    maintainer_email='viktor@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_scenario = marv_scenarios.example_scenario:main',
            'test_throttle = marv_scenarios.test_throttle:main',
            'test_constant_velocity = marv_scenarios.test_constant_velocity:main',
            'set_reference_position = marv_scenarios.set_reference_position:main',
            'remote_network_steering = marv_scenarios.remote_network_steering:main',
            'waypoint_navigation = marv_scenarios.waypoint_navigation:main',
            'thrust_controller_test = marv_scenarios.thrust_controller_test:main',
            'heading_controller_test = marv_scenarios.heading_controller_test:main',
	    ],
    },
)
