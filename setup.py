from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'nartech_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['main', 'mettabridge'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'start'), ['start/nartech_view.rviz']),
    ],
    package_data={
        'channels': ['*.weights', '*.cfg', '*.names'],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nartech',
    maintainer_email='nartech@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nartech_main = nartech_ros.nartech_main:main',
            'cmd_vel_adapter = nartech_ros.cmd_vel_adapter:main',
            'scan_mux_selector = nartech_ros.scan_mux_selector:main',
        ],
    },
)
