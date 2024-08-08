from setuptools import find_packages, setup

package_name = 'doteacher_control'

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
    maintainer='ssafy',
    maintainer_email='boutljy@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = doteacher_control.motor_controller_node:main',
            'ackermann_steering_controller = doteacher_control.ackermann_steering_controller:main',
            'motor_control_node = doteacher_control.motor_control_node:main'
        ],
    },
)
