from setuptools import setup
import os
from glob import glob

package_name = 'voice_assistant'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
<<<<<<< Updated upstream
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
=======
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml'))
>>>>>>> Stashed changes
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Barza Nisar',
    maintainer_email='barzanisar93@gmail.com',
    description='Chats with user and handles voice commands',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_assistant_node = voice_assistant.voice_assistant_node:main',
            'snack_wanted_service = voice_assistant.snack_wanted_service:main'
        ],
    },
)
