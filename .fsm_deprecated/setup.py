import os
from glob import glob
from setuptools import setup

package_name = 'fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dynamight',
    maintainer_email='danielwjkhoo@gmail.com',
    description='Finite State Machine and Navigation stack monitor node.',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # "trailbot_fsm = fsm.trailbot_fsm:main",
            "trailbot_fsm = fsm.trailnav_fsm:main",
            "navigator_cmd_node = fsm.trailnav_cmd:main",
            "fsm_test = fsm.nav_fsm_test_file:main",
            "navigator_node = fsm.navigator_node:main",
        ],
    },
)
