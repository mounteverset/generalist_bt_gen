from glob import glob
from pathlib import Path
import sys

from setuptools import find_packages, setup

with open('/tmp/user_interface_setup_args.log', 'a', encoding='utf-8') as f:
    f.write(str(sys.argv) + '\n')

def _strip_arg(flag: str):
    while flag in sys.argv:
        sys.argv.remove(flag)

_strip_arg('--editable')
_strip_arg('--uninstall')
_strip_arg('--build-directory')

if len(sys.argv) >= 2 and sys.argv[1] == 'develop':
    new_args = [sys.argv[0], 'develop']
    injected = False
    default_install_dir = Path('build/user_interface/prefix_override')
    for arg in sys.argv[2:]:
        if arg == 'symlink_data':
            continue
        if arg.startswith('/') and not injected:
            # skip original build directory path
            continue
        new_args.append(arg)
    if not injected:
        new_args.extend(['--install-dir', str(default_install_dir)])
    sys.argv = new_args
    with open('/tmp/user_interface_setup_args_filtered.log', 'a', encoding='utf-8') as f:
        f.write(str(sys.argv) + '\n')

from setuptools.command.develop import develop as _develop

class DevelopCommand(_develop):
    def check_site_dir(self):
        return

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
    install_requires=[
        'setuptools',
        'rich',
        'fastapi',
        'uvicorn',
        'jinja2',
    ],
    cmdclass={'develop': DevelopCommand},
    entry_points={
        'console_scripts': [
            'chat_node = user_interface.chat_node:main',
            'web_ui_node = user_interface.web_ui_node:main',
        ],
    },
)
