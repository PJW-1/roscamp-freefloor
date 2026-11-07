from glob import glob

from setuptools import setup

setup(
    name='fleet_gz_sim',
    version='0.1.0',
    packages=['fleet_gz_sim'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fleet_gz_sim']),
        ('share/fleet_gz_sim', ['package.xml']),
        ('share/fleet_gz_sim/launch', glob('launch/*.launch.py')),
        ('share/fleet_gz_sim/urdf', glob('urdf/*.xacro')),
        ('share/fleet_gz_sim/worlds', glob('worlds/*.sdf')),
        ('share/fleet_gz_sim/maps', glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'drive_check = fleet_gz_sim.drive_check:main',
    ]},
)
