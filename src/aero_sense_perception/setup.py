import os

from setuptools import setup

PACKAGE = "aero_sense_perception"


def tree(src):
    return [(os.path.join("share", PACKAGE, root), [os.path.join(root, f) for f in files])
            for root, _, files in os.walk(src) if files]


setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ] + tree("config") + tree("models"),       # models/: the RGB detector's weights, from ml/models
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kushsaraf",
    maintainer_email="teamdronematrx@gmail.com",
    description="Finds casualties in the drone's sensor stream: thermal and RGB detection, geolocation, tracking.",
    license="TODO",
    entry_points={"console_scripts": [
        f"victim_detector = {PACKAGE}.victim_detector:main",
        f"rgb_detector = {PACKAGE}.rgb_detector:main",
        f"hazard_mapper = {PACKAGE}.hazard_mapper:main",
    ]},
)
