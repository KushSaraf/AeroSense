from setuptools import setup

PACKAGE = "aero_sense_mission"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kushsaraf",
    maintainer_email="codenexusky@gmail.com",
    description="Aero Sense autopilot adapter, drone interface and mission state machine.",
    license="TODO",
    entry_points={"console_scripts": [
        f"drone_interface = {PACKAGE}.drone_interface:main",
        f"mission_manager = {PACKAGE}.mission_manager:main",
        f"comms_link = {PACKAGE}.comms_link:main",
        f"gps_jammer = {PACKAGE}.gps_jammer:main",
        f"ground_routes = {PACKAGE}.ground_routes:main",
    ]},
)
