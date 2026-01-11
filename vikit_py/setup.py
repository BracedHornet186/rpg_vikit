from setuptools import setup
import os
from glob import glob

package_name = 'vikit_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], 
    package_dir={'': 'src'}, 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_node = ros_node.ros_node:main',
        ],
    },
)