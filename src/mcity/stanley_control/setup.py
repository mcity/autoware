from setuptools import setup
import os
from glob import glob

package_name = 'stanley_control'

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
    description='Stanley lateral controller with PI speed control',
    license='TODO',
    entry_points={
        'console_scripts': [
            'stanley_control = stanley_control.stanley_control_node:main',
        ],
    },
)
