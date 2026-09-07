"""ament_python package: nuway_carla_bridge (M0)."""

from pathlib import Path

from setuptools import find_packages, setup

PACKAGE = "nuway_carla_bridge"
setup(
    name=PACKAGE,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/config", [str(p) for p in Path("config").glob("*.yaml")]),
        (
            f"share/{PACKAGE}/launch",
            [str(p) for p in Path("launch").glob("*.launch.py")],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tetsui Ohkubo",
    maintainer_email="peryaudo@gmail.com",
    description="CARLA side of nuway: world_manager and control_adapter nodes",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"world_manager = {PACKAGE}.world_manager:main",
            f"control_adapter = {PACKAGE}.control_adapter:main",
        ],
    },
)
