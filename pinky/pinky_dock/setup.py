from setuptools import setup

setup(
    name='pinky_dock',
    version='0.1.0',
    packages=['pinky_dock'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pinky_dock']),
        ('share/pinky_dock', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [

    ]},
)
