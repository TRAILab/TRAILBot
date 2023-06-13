from setuptools import setup

package_name = 'trailbot_main'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stevenw',
    maintainer_email='stevenw@utias.utoronto.ca',
    description='Top-level package to house demo launch files',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
