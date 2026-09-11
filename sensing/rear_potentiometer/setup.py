import os
from setuptools import find_packages, setup

try:
    from common_python.setup_util import get_data_files
except ImportError:
    def get_data_files(pkg, target_dirs=()):
        data = [
            ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
            (os.path.join('share', pkg), ['package.xml']),
        ]
        for td in target_dirs:
            for root, _, files in os.walk(td):
                for f in files:
                    data.append((os.path.join('share', pkg, root), [os.path.join(root, f)]))
        return data

package_name = 'rear_potentiometer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=get_data_files(package_name, ("launch",)),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='issaomura',
    maintainer_email='issa_omura@jp.honda',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rear_potentiometer = rear_potentiometer.rear_potentiometer:main',
        ],
    },
)
