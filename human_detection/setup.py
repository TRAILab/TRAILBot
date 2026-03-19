from setuptools import setup, find_packages

package_name = 'human_detection'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhaodong',
    maintainer_email='zhaodong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    #extras_require={
    #    'test': ['pytest'],
    #},
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_detection_node = human_detection.human_detection_node:main',
            'nav2_path_publisher = human_detection.nav2_path_publisher:main',
            'simple_path_publisher = human_detection.simple_path_publisher:main',
            'pose_publisher = human_detection.pose_publisher:main',
        ],
    },
)
