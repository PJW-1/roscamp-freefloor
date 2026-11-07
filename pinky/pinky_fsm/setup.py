from glob import glob

from setuptools import setup

setup(
    name='pinky_fsm',
    version='0.1.0',
    packages=['pinky_fsm'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pinky_fsm']),
        ('share/pinky_fsm', ['package.xml']),
        ('share/pinky_fsm/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'pinky_fsm_node = pinky_fsm.node:main',
    ]},
)
