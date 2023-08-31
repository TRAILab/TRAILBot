from setuptools import setup

package_name = 'trail_detection_node'

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
    maintainer='zhaodong',
    maintainer_email='tom2352759619@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trail_detection = trail_detection_node.v2_trailDetectionNode:main',
            'visualizer = trail_detection_node.visualizer_v2_trailDetectionNode:main',
            'original_visualizer = trail_detection_node.visualizer_v1_trailDetectionNode:main',
        ],
    },
)
