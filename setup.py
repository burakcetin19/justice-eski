from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'justice'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    include_package_data=True,
    zip_safe=True,
    maintainer='HAVK Takımı',
    maintainer_email='havk@todo.todo',
    description='GPS + Görsel Hibrit Takip Sistemi — L1 Guidance & YOLO+TCTrack++ Birleşik Paket',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'justice_node = justice.justice_node:main',
            'tracker_node = justice.tracking.tracker_node:main',
            'mock_target_receiver = justice.mock_target_receiver:main',
        ],
    },
)
