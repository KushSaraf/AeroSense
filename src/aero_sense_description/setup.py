import os

from setuptools import setup

PACKAGE = "aero_sense_description"


def tree(src):
    """data_files entries installing every file under `src`, keeping the directory layout."""
    return [(os.path.join("share", PACKAGE, root), [os.path.join(root, f) for f in files])
            for root, _, files in os.walk(src) if files]


setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ] + tree("config") + tree("templates") + tree("meshes"),
    install_requires=["setuptools", "jinja2", "pyyaml"],
    zip_safe=True,
    maintainer="kushsaraf",
    maintainer_email="codenexusky@gmail.com",
    description="Aero Sense airframe and sensor payload models.",
    license="TODO",
)
