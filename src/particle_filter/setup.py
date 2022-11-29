import os
from glob import glob
from setuptools import setup

package_name = 'particle_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'map'), glob('map/*map*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav Agrahari',
    maintainer_email='abhi.agrahari@uwaterloo.ca',
    description='Particle Filter Algorithm',
    license='MTI',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'particle_filter = particle_filter.run_particle_filter:main'
        ],
    },
)
