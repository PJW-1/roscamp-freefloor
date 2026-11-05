from setuptools import setup

setup(
    name='pinky_fsm',
    version='0.1.0',
    packages=['pinky_fsm'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pinky_fsm']),
        ('share/pinky_fsm', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [

    ]},
)
