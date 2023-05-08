from setuptools import setup

package_name = 'voice_assistant_pkg'

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
    description='Initiates voice assistant/ChatGPT and upon user voice commands calls runservo service  to operate servos',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chatbot = voice_assistant_pkg.voice_assistant:main'
        ],
    },
)
