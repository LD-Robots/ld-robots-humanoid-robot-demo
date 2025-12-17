from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pinocchio_balance_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alexzamfir94@gmail.com',
    description='Advanced balance control using Pinocchio dynamics',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pinocchio_balance_controller.py = pinocchio_balance_control.pinocchio_balance_controller:main',
        ],
    },
)
