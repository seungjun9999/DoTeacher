from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'usb_cam_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/usb_cam_launch.py']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),  # Include resource files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeongjae',
    maintainer_email='jeongjae@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "usbcam_node = usb_cam_pkg.get_video:main",
            "display_node = usb_cam_pkg.show_video:main",
            "detect_marker_node = usb_cam_pkg.detect_marker:main",
            "detect_pose_node = usb_cam_pkg.detect_pose:main"
        ],
    },
)
