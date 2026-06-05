from setuptools import find_packages, setup
import os
from glob import glob

package_name = "xarm_joint_bridge"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your@email.com",
    description="Bridges real xArm7 joint_states into namespaced topics for Isaac Sim",
    license="MIT",
    entry_points={
        "console_scripts": [
            "joint_bridge_node = xarm_joint_bridge.joint_bridge_node:main",
        ],
    },
)
