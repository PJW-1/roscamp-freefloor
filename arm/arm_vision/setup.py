from setuptools import setup

setup(
    name='arm_vision',
    version='0.1.0',
    packages=['arm_vision'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/arm_vision']),
        ('share/arm_vision', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [

    ]},
)
