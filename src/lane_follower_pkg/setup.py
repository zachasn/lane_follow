import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lane_follower_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.sdf'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.dae')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zach',
    maintainer_email='hassa844@umn.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = lane_follower_pkg.controller_node:main',
            'vision_node = lane_follower_pkg.vision_node:main',
        ],
    },
)
