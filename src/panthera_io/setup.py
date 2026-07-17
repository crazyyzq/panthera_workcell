from glob import glob
import os

from setuptools import setup


package_name = 'panthera_io'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panthera',
    maintainer_email='user@example.com',
    description='Configurable digital IO bridge for Panthera workflows',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpio_io_node = panthera_io.gpio_io_node:main',
        ],
    },
)
