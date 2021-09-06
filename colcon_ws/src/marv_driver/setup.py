from setuptools import setup
import os
from glob import glob

package_name = 'marv_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name, ['launch/marv_driver.launch.py']),
        ('share/' + package_name, ['resource/marv.dbc']),
        ('lib/' + package_name, ['library/timer.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
        #(os.path.join('lib', package_name, 'library'), glob('library/*.py'))
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
            'power_management = marv_driver.marv_power_management:main',
            'can_bridge = marv_driver.marv_can_bridge:main',
            'status_sender = marv_driver.marv_status_sender:main',
            'sbg_interface = marv_driver.marv_sbg_interface:main',
            'logger = marv_driver.marv_logger:main',
            'scenario_handler = marv_driver.marv_scenario_handler:main',
            'heartbeat_acu = marv_driver.marv_heartbeat_acu:main',
            'scenario_starter = marv_driver.marv_scenario_starter:main',
	    ],
    },
)
