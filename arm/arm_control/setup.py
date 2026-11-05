from setuptools import setup

setup(
    name='arm_control',
    version='0.1.0',
    packages=['arm_control'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/arm_control']),
        ('share/arm_control', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [

    ]},
)
