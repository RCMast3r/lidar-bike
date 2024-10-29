from setuptools import setup

package_name = 'driver_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/launch', ['launch/multi_node_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ben Hall',
    maintainer_email='bhall75@gatech.edu',
    description='A package to launch multiple nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
