from setuptools import setup

package_name = "jonny5_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="corgiolu-labs",
    maintainer_email="dev@corgiolu-labs.local",
    description="Dry-run simulators for local JONNY5 ROS2 development without robot hardware.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "teleop_intent_sim_node = jonny5_sim.teleop_intent_sim_node:main",
        ],
    },
)