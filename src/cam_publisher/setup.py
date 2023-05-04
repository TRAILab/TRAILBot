from setuptools import setup

package_name = 'cam_publisher'

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
    maintainer_email='zhaodong@todo.todo',
    description='Publish the image from webcam',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam = cam_publisher.camPublisher:main'
        ],
    },
)
