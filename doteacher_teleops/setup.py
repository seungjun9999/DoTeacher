from setuptools import find_packages, setup

package_name = 'doteacher_teleops'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='d102',
    maintainer_email='d102@ssafy.com',
    description='Teleoperation package for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'teleops_joy_node = doteacher_teleops.teleops_joy_node:main'
        ],
    },
)
