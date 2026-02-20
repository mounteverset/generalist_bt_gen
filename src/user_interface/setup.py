from glob import glob

from setuptools import find_packages, setup

package_name = 'user_interface'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    # Ensure template HTML files are installed with the Python package
    include_package_data=True,
    package_data={
        'user_interface': ['templates/*.html'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luke',
    maintainer_email='Lukas.E@web.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'chat_node = user_interface.chat_node:main',
            'web_ui_node = user_interface.web_ui_node:main',
        ],
    },
)
