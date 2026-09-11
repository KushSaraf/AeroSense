from setuptools import setup

PACKAGE = "aero_sense_navigation"

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
    maintainer_email="teamdronematrx@gmail.com",
    description="Keeps the drone off the obstacles it can see.",
    license="TODO",
    entry_points={"console_scripts": [f"obstacle_guard = {PACKAGE}.obstacle_guard:main"]},
)
