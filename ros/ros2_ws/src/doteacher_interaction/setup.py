from setuptools import find_packages, setup

package_name = 'doteacher_interaction'

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
    maintainer='jetson',
    maintainer_email='dev.ocft@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coordinate_listener = doteacher_interaction.coordinate_listener:main',
            'rpi_cmd_publisher_service = doteacher_interaction.rpi_cmd_publisher_service:main',
        ],
    },
)
