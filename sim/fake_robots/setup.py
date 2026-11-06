from glob import glob

from setuptools import find_packages, setup

setup(
    name='fake_robots',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fake_robots']),
        ('share/fake_robots', ['package.xml']),
        ('share/fake_robots/launch', glob('launch/*.py')),
        ('share/fake_robots/scenarios', glob('scenarios/*.yaml')),
        ('share/fake_robots/maps', glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'fake_pinky_node = fake_robots.fake_pinky_node:main',
        'fake_workcell_node = fake_robots.fake_workcell_node:main',
        'fake_arm_node = fake_robots.fake_arm_node:main',
        'fake_vision_node = fake_robots.fake_vision_node:main',
        'fake_central_node = fake_robots.fake_central_node:main',
        'sim_clock_node = fake_robots.sim_clock_node:main',
        'scenario_monitor_node = fake_robots.scenario_monitor_node:main',
    ]},
)
