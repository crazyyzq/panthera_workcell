from glob import glob
import os

from setuptools import setup

package_name = 'panthera_web_hmi'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'static'), glob('static/*.*')),
        (os.path.join('share', package_name, 'static', 'assets'), glob('static/assets/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tk',
    maintainer_email='153282466@qq.com',
    description='Web HMI bridge for Panthera spectrometer workcell using ROS 2 topics and services.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_hmi_node = panthera_web_hmi.web_hmi_node:main',
        ],
    },
)
