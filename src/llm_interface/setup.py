from setuptools import find_packages, setup

package_name = 'llm_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'implementation_plan.md', 'service_definition_plan.md']),
        ('share/' + package_name + '/config', ['config/llm_interface_params.yaml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'langchain',
        'langchain-google-genai',
    ],
    zip_safe=True,
    maintainer='luke',
    maintainer_email='Lukas.E@web.de',
    description='LangChain-based LLM orchestrator that provides subtree planning services to the mission coordinator.',
    license='Apache-2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_interface_node = llm_interface.node:main',
        ],
    },
)
