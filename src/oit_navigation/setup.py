import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'oit_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Vision-Only BEV Navigation, YOLOP Lane Detection, and Pure Pursuit Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = oit_navigation.video_publisher:main',
            'yolop_lane_detector = oit_navigation.yolop_lane_detector:main',
            'bev_pure_pursuit_node = oit_navigation.bev_pure_pursuit_node:main',
            'bev_lane_tracker = oit_navigation.bev_pure_pursuit_node:main',
            'pure_pursuit_controller = oit_navigation.bev_pure_pursuit_node:main',
            'traffic_light_distance_node = oit_navigation.traffic_light_distance_node:main',
            'verification_gui = oit_navigation.verification_gui:main',
        ],
    },
)
