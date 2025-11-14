from setuptools import find_packages, setup

package_name = 'mission_coordinator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'implementation_plan.md']),
        ('share/' + package_name + '/config', ['config/mission_coordinator_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luke',
    maintainer_email='Lukas.E@web.de',
    description='Mission control node coordinating UI, LLM, and bt_executor.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_coordinator_node = mission_coordinator.node:main',
        ],
    },
)
