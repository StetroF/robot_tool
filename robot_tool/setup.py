from setuptools import setup
import os
from glob import glob

package_name = 'robot_tool'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include documentation
        (os.path.join('share', package_name, 'docs'), glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='A standardized robot controller interface package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jaka_controller_node = robot_tool.jaka_controller:main',
            'robot_fastapi_server = robot_tool.robot_fastapi:main',
        ],
    },
) 