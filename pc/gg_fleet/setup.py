from setuptools import setup

setup(
    name='gg_fleet',
    version='0.1.0',
    packages=['gg_fleet'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/gg_fleet']),
        ('share/gg_fleet', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'fleet_manager_node = gg_fleet.fleet_manager_node:main',
    ]},
)
