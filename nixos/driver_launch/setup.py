from setuptools import find_packages
from setuptools import setup

package_name = 'meta_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/meta_launch', ['meta_launch/multi_node_launch.py']),
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
