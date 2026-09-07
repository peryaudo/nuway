"""ament_python package: nuway_rclpy (M0)."""

from setuptools import find_packages, setup

PACKAGE = "nuway_rclpy"
setup(
    name=PACKAGE,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tetsui Ohkubo",
    maintainer_email="peryaudo@gmail.com",
    description="rclpy-side helpers shared by the Python node packages",
    license="MIT",
)
