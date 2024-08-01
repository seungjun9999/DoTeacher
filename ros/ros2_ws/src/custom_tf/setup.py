from setuptools import find_packages, setup

package_name = 'custom_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/custom_tf_launch.py']),
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
            'custom_tf_node = custom_tf.custom_tf_node:main',
            'custom_tf_slam_node = custom_tf.custom_tf_slam_node:main',
            'custom_tf_base_node = custom_tf.custom_tf_base_node:main'
        ],
    },
)
