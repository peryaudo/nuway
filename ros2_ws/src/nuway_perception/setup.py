"""ament_python package: nuway_perception (M0)."""

from pathlib import Path

from setuptools import find_packages, setup

PACKAGE = "nuway_perception"
setup(
    name=PACKAGE,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/config", [str(p) for p in Path("config").glob("*.yaml")]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tetsui Ohkubo",
    maintainer_email="peryaudo@gmail.com",
    description="Perception nodes: GT twins (M0), learned perception (M3/M4)",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"gt_perception_node = {PACKAGE}.gt_perception_node:main",
            f"gt_traffic_light_node = {PACKAGE}.gt_traffic_light_node:main",
        ],
    },
)
