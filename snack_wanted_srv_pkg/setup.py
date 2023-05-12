from setuptools import setup

package_name = 'snack_wanted_srv_pkg'

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
    description='Temporary service node for snack_wanted srv',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'snack_wanted_service = snack_wanted_srv_pkg.snack_wanted_srv:main'
        ],
    },
)
