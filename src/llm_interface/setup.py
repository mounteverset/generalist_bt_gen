from setuptools import setup

package_name = "llm_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="luke",
    maintainer_email="Lukas.E@web.de",
    description="Service and helpers for interacting with an LLM to generate Behavior Trees.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "llm_service_node = llm_interface.llm_service_node:main",
        ],
    },
)
