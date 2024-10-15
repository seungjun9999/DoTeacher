from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'doteacher_backend'

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
    maintainer='jeongjae',
    maintainer_email='npcocft@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client_backend_db_node = doteacher_backend.client_backend_db_node:main',
            'client_backend_log_node = doteacher_backend.client_backend_log_node:main',
            'server_backend_op_node = doteacher_backend.server_backend_op_node:main',
            'client_backend_op_node = doteacher_backend.client_backend_op_node:main',
            'upload_picture = doteacher_backend.upload_picture:main',
            'detect_pose_take_picture = doteacher_backend.detect_pose_take_picture:main',
            'upload_picture_service = doteacher_backend.upload_picture_service:main',
            
            'connect_server_node = doteacher_backend.connect_server_node:main',
        ],
    },
)
