from setuptools import find_packages, setup

package_name = 'eval'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/eval.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parker',
    maintainer_email='psbradsh@umich.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eval_node = eval.eval_node:main',
            'gt_publisher_node = eval.gt_publisher_node:main',
        ],
    },
)
