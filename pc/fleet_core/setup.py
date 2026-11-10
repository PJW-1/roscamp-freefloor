from setuptools import setup

setup(
    name='fleet_core',
    version='0.1.0',
    packages=['fleet_core', 'fleet_core.arm', 'fleet_core.common', 'fleet_core.orders',
              'fleet_core.traffic', 'fleet_core.traffic.floor'],
    package_data={'fleet_core': ['data/*.yaml', 'data/*.pgm']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fleet_core']),
        ('share/fleet_core', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [

    ]},
)
