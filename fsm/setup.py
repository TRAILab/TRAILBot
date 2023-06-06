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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dynamight',
    maintainer_email='danielwjkhoo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "trailbot_action = fsm.trailbot_action:main",
            "trailbot_fsm = fsm.trailbot_fsm:main",
            "basic_navigator = fsm.basic_navigator:main",
            "navigator = fsm.navigator:main",
            "fsm_node = fsm.fsm_node:main",
            "tf2_listener = fsm.tf2_listener:main",
        ],
    },
)
