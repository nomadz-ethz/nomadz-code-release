from glob import glob

from setuptools import setup

package_name = "nomadz_webots_controller"
data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name + "/launch", glob("launch/*.py")))
data_files.append(("share/" + package_name + "/worlds", glob("worlds/*.wbt")))
data_files.append(("share/" + package_name + "/resource", glob("resource/*.urdf")))
data_files.append(
    (
        "share/" + package_name + "/resource/controllers/nao_lola_python",
        glob("resource/controllers/nao_lola_python/*.py"),
    )
)
data_files.append(
    ("share/" + package_name + "/resource/protos", glob("resource/protos/*.proto"))
)
data_files.append(
    ("share/" + package_name + "/resource/textures", glob("resource/textures/*"))
)
data_files.append(("share/" + package_name, ["package.xml"]))

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team NomadZ",
    maintainer_email="nomadz@list.ee.ethz.ch",
    description="This package allows the user to use both webots nao lola controller"
    "as well as the webots_ros2 APIs to interactive with the webots simulation environment",
    license="MIT",
    entry_points={
        "console_scripts": [
            "nao_webots_driver = nomadz_webots_controller.nao_webots_driver:main",
            "supervisor = nomadz_webots_controller.supervisor:main",
        ],
    },
)
