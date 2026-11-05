from setuptools import setup

setup(
    name='central',
    version='0.1.0',
    packages=['central'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/central']),
        ('share/central', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={'console_scripts': [

    ]},
)
