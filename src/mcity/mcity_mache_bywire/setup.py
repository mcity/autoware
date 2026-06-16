from setuptools import setup

package_name = 'mcity_mache_bywire'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mcity_mache_bywire.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raj',
    maintainer_email='raj@umich.edu',
    description='The mcity_mache_bywire package (Python)',
    license='To Commercialize',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcity_mache_bywire = mcity_mache_bywire.mcity_bywire_node:main',
        ],
    },
)
