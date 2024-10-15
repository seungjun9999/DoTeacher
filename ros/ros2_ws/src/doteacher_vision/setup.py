from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'doteacher_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('*.pt')) # 수정 필요: 복사 안 됨
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
            'take_picture = doteacher_vision.take_picture:main',
            
            'detect_picture_service = doteacher_vision.detect_picture_service:main',
        ],
    },
)
