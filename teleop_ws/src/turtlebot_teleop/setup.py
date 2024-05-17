from setuptools import find_packages, setup

package_name = "turtlebot_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Eduardo Barreto",
    maintainer_email="eduardopontobarreto@gmail.com",
    description="Teleoperation package for TurtleBot",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_node = turtlebot_teleop.teleop_node:main",
        ],
    },
)
