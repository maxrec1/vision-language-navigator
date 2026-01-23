from setuptools import setup
import os
from glob import glob

package_name = 'tb4_gz_rqt_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools', 'requests'],
    entry_points={
        'console_scripts': [
            'parse_command_node = tb4_gz_rqt_launch.command_parser_node:main',
        ],
    },
    zip_safe=False,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Launch TurtleBot4 in Gazebo and start rqt_image_view',
    license='Apache-2.0',
    classifiers=['Programming Language :: Python :: 3'],
)
