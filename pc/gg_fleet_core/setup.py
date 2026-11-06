from setuptools import setup

setup(
    name='gg_fleet_core',
    version='0.1.0',
    packages=['gg_fleet_core', 'gg_fleet_core.common', 'gg_fleet_core.orders',
              'gg_fleet_core.traffic'],
    package_data={'gg_fleet_core': ['data/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/gg_fleet_core']),
        ('share/gg_fleet_core', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [

    ]},
)
