import os
from glob import glob
from setuptools import setup

package_name = 'tf_transform'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #include node in src
	(os.path.join('share', package_name, 'src'), glob('src/tf_transform_node.py')),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koener',
    maintainer_email='nic.koenig37@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'tf_transform_node = tf_transform.tf_transform_node:main',
 
        ],
    },
)
