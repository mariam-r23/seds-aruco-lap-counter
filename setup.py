from setuptools import setup

package_name = "aruco_lap_counter"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/lap_counter.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mariam Raza",
    maintainer_email="mariam.raza2314@gmail.com",
    description="ArUco gate detection and lap counting node",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lap_counter_node = aruco_lap_counter.lap_counter_node:main",
            "image_folder_publisher = aruco_lap_counter.image_folder_publisher:main",
        ],
    },
)
