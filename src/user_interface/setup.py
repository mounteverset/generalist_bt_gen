from glob import glob
from pathlib import Path
import sys

from setuptools import find_packages, setup

# Colcon may pass --editable when using --symlink-install; older setuptools commands
# do not recognize it. Remove to keep setup.py compatible.
if '--editable' in sys.argv:
    sys.argv.remove('--editable')

package_name = 'user_interface'
share_dir = Path('share') / package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={'user_interface': ['templates/*.html']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (str(share_dir), ['package.xml', 'plan_chat_interface.md']),
        (str(share_dir / 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'rich', 'fastapi', 'uvicorn', 'jinja2'],
    zip_safe=True,
    maintainer='luke',
    maintainer_email='Lukas.E@web.de',
    description='Terminal + web chat UIs for configuring commands sent to the LangChain/BT mission coordinator.',
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
