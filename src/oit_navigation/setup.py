import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'oit_navigation'

setup(
    name=package_name,
    version='0.2.0',
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
    description='Lane detection (YOLOP/UFLD), lap mapping and QP raceline driving, traffic light distance',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = oit_navigation.video_publisher:main',
            'lane_detector = oit_navigation.lane_detector:main',
            'odom_imu_localizer = oit_navigation.odom_imu_localizer:main',
            'lane_navigator = oit_navigation.lane_navigator_node:main',
            'traffic_light_distance_node = oit_navigation.traffic_light_distance_node:main',
            'image_compressor_node = oit_navigation.image_compressor_node:main',
            'export_tensorrt = oit_navigation.export_tensorrt:main',
            'export_onnx_web = oit_navigation.export_onnx_web:main',
            'export_ufld_onnx = oit_navigation.export_ufld_onnx:main',
            'verification_gui = oit_navigation.verification_gui:main',
        ],
    },
)
