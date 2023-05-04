from setuptools import setup

package_name = 'cam_subscriber'

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
    description='Receive the img, process, and show',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'det2 = cam_subscriber.camSubscriber:main'
        ],
    },
)
