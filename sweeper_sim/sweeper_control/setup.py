from setuptools import setup, find_packages
package_name = 'sweeper_control'
setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/control.launch.py']),
        ('share/' + package_name + '/config', ['config/control.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sweeper',
    maintainer_email='dev@sweeper.local',
    description='MPC controller (CILQR) + Pure Pursuit fallback for the sweeper.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'controller_node = sweeper_control.controller_node:main',
        'mpc_controller_node = sweeper_control.mpc_controller_node:main',
    ]},
)
