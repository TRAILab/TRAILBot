from setuptools import setup

package_name = 'voice_assistant'

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
    maintainer='Barza Nisar',
    maintainer_email='barzanisar93@gmail.com',
    description='Chats with user and handles voice commands',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_assistant_node = voice_assistant.voice_assistant_node:main',
            'voice_arduino_bridge_node = voice_assistant.voice_arduino_bridge_node:main'
        ],
    },
)
