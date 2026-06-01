from setuptools import setup

package_name = "jonny5_teleop_vr"

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
    description="ROS2 bridge for JONNY5 WebXR/WebSocket teleoperation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ws_teleop_bridge_node = jonny5_teleop_vr.ws_teleop_bridge_node:main",
        ],
    },
)
