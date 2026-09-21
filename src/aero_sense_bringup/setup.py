from glob import glob

from setuptools import setup

PACKAGE = "aero_sense_bringup"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kushsaraf",
    maintainer_email="codenexusky@gmail.com",
    description="Launch files, configuration and diagnostics for the Aero Sense SAR simulation.",
    license="TODO",
    entry_points={"console_scripts": [
        f"system_check = {PACKAGE}.system_check:main",
        f"stop_sim = {PACKAGE}.stop_sim:main",
        f"spawn = {PACKAGE}.spawn:main",
        f"rangefinder_sim = {PACKAGE}.rangefinder_sim:main",
        f"gas_sim = {PACKAGE}.gas_sim:main",
    ]},
)
