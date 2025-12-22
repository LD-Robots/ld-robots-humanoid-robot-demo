from setuptools import setup

package_name = 'wbc_pinocchio_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/wbc_controller.yaml',
            'config/initial_pose.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/wbc_mujoco.launch.py',
            'launch/wbc_full_mujoco.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Quasi-static WBC controller using Pinocchio (optional Crocoddyl) for MuJoCo humanoid.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wbc_controller = wbc_pinocchio_controller.wbc_controller:main',
        ],
    },
)
