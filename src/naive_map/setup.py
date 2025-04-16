from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'naive_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parker',
    maintainer_email='parker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'naive_mapper_node = naive_map.naive_mapper_node:main',
            'simple_odom_node = naive_map.simple_odom_node:main',
        ],
    },
)
