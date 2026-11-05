from setuptools import setup

setup(
    name='fleet_manager',
    version='0.1.0',
    packages=['fleet_manager'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fleet_manager']),
        ('share/fleet_manager', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'fleet_manager_node = fleet_manager.fleet_manager_node:main',
    ]},
)
