from setuptools import find_packages, setup

package_name = "nomadz_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pynput"],
    zip_safe=True,
    maintainer="Team NomadZ",
    maintainer_email="nomadz@list.ee.ethz.ch",
    description="Teleoperation package for controlling the robot via MotionRequest message.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_teleop_node = nomadz_teleop.keyboard_teleop_node:main",
        ]
    },
)
