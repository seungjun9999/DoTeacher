from setuptools import find_packages, setup

package_name = 'doteacher_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'camera_publisher',
        'image_processor',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeongjae',
    maintainer_email='npcocft@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = doteacher_vision.camera_publisher:main',
            'image_processor = doteacher_vision.image_processor:main',
        ],
    },
)
