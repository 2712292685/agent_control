import glob
import os

from setuptools import setup

package_name = "agent_control_api"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="snail",
    maintainer_email="snail@todo.todo",
    description="HTTP/JSON API and driver TCP bridge for agent_control.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "agent_control_api = agent_control_api.api_node:main",
        ],
    },
)
