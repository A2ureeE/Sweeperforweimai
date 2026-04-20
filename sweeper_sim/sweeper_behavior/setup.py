from setuptools import setup
package_name = 'sweeper_behavior'
setup(
    name=package_name, version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/behavior.launch.py']),
        ('share/' + package_name + '/config', ['config/behavior.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sweeper', maintainer_email='dev@sweeper.local',
    description='FSM/BT arbiter.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'behavior_node = sweeper_behavior.behavior_node:main',
    ]},
)
