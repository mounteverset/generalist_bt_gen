from glob import glob

from setuptools import find_packages, setup

package_name = 'mission_reasoner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luke',
    maintainer_email='Lukas.E@web.de',
    description='Pre-selection mission capability validator.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mission_reasoner_node = mission_reasoner.node:main',
        ],
    },
)
