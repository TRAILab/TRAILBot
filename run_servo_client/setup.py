from setuptools import setup

package_name = 'run_servo_client'

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
    maintainer='barza',
    maintainer_email='barzanisar93@gmail.com',
    description='Client to runservo service',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_servo_client = run_servo_client.run_servo_client:main'
        ],
    },
)
