from setuptools import setup

package_name = 'sweeper_planning'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planning.launch.py']),
        ('share/' + package_name + '/config', ['config/planning.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sweeper',
    maintainer_email='dev@sweeper.local',
    description='Coverage path planning, local planner, scoring and mission tracking.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coverage_node      = sweeper_planning.coverage_node:main',
            'planner_node       = sweeper_planning.planner_node:main',
            'score_logger_node  = sweeper_planning.score_logger_node:main',
            'mission_runner_node = sweeper_planning.mission_runner_node:main',
        ],
    },
)
