from glob import glob

from setuptools import find_packages, setup

setup(
    name='gg_sim',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/gg_sim']),
        ('share/gg_sim', ['package.xml']),
        ('share/gg_sim/launch', glob('launch/*.py')),
        ('share/gg_sim/scenarios', glob('scenarios/*.yaml')),
        ('share/gg_sim/maps', glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'fake_pinky_node = gg_sim.fake_pinky_node:main',
        'fake_workcell_node = gg_sim.fake_workcell_node:main',
        'fake_arm_node = gg_sim.fake_arm_node:main',
        'fake_vision_node = gg_sim.fake_vision_node:main',
        'fake_central_node = gg_sim.fake_central_node:main',
        'sim_clock_node = gg_sim.sim_clock_node:main',
        'scenario_monitor_node = gg_sim.scenario_monitor_node:main',
    ]},
)
