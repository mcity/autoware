from setuptools import setup
import os
from glob import glob

package_name = 'pursuit_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raj',
    maintainer_email='rpatnaik@umich.edu',
    description='Pure Pursuit lateral controller with cascaded PID speed control',
    license='TODO',
    entry_points={
        'console_scripts': [
            'pursuit_control = pursuit_control.pursuit_control_node:main',
        ],
    },
)
