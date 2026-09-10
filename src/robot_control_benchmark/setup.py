from glob import glob

from setuptools import find_packages, setup

package_name = "robot_control_benchmark"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        (
            "share/" + package_name + "/models/differential_drive",
            glob("models/differential_drive/*"),
        ),
    ],
    install_requires=["setuptools", "numpy", "scipy", "matplotlib", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="robot_control_benchmark maintainers",
    maintainer_email="maintainer@example.com",
    description="Reproducible trajectory-controller benchmark for differential-drive robots.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "benchmark = robot_control_benchmark.cli:main",
            "controller_node = robot_control_benchmark.ros_node:main",
        ],
    },
)

