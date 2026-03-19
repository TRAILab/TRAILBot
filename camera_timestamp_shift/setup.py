from setuptools import find_packages, setup

package_name = 'camera_timestamp_shift'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trailbot',
    maintainer_email='nic.koenig37@gmail.com',
    description='Online timestamp shift for camera using LiDAR reference',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_timestamp_shift = camera_timestamp_shift.camera_timestamp_shift_node:main',
        ],
    },
)
