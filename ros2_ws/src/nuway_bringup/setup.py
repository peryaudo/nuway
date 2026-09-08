"""ament_python package: nuway_bringup (M0)."""

from pathlib import Path

from setuptools import find_packages, setup

PACKAGE = "nuway_bringup"
setup(
    name=PACKAGE,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (
            f"share/{PACKAGE}/launch",
            [str(p) for p in Path("launch").glob("*.launch.py")],
        ),
    ],
    install_requires=["setuptools"],
    # colcon runs `python -m pytest` only for packages that declare it
    # (otherwise it falls back to unittest discovery and exits 5 on an
    # empty package).
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Tetsui Ohkubo",
    maintainer_email="peryaudo@gmail.com",
    description="The stack launch file and its per-subsystem launches",
    license="MIT",
)
