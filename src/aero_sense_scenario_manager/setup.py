import os

from setuptools import setup

PACKAGE = "aero_sense_scenario_manager"


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
    ] + tree("config"),
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="kushsaraf",
    maintainer_email="teamdronematrx@gmail.com",
    description="Scenario content for the Aero Sense simulation: victims, and their ground truth.",
    license="TODO",
    entry_points={"console_scripts": [
        f"victim_ground_truth = {PACKAGE}.ground_truth:main",
    ]},
)
