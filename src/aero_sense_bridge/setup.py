from setuptools import setup

PACKAGE = "aero_sense_bridge"

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
    description="Serves the live mission state to the web dashboard over HTTP and WebSocket.",
    license="TODO",
    entry_points={"console_scripts": [f"dashboard_bridge = {PACKAGE}.dashboard_bridge:main"]},
)
