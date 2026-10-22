from glob import glob
from setuptools import find_packages, setup

setup(
    name="tray_workcell",
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/tray_workcell"]),
        ("share/tray_workcell", ["package.xml"]),
        *[("share/tray_workcell/" + directory, glob(directory + "/*.*"))
          for directory in ("launch", "config", "urdf", "rviz")],
    ],
    install_requires=["setuptools", "PyYAML"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Tray Workcell contributors",
    maintainer_email="maintainer@example.com",
    description="Python ROS 2 UR5e/Robotiq tray packing with MoveIt 2 and Gazebo",
    license="BSD-3-Clause",
    entry_points={"console_scripts": [
        "pick_place = tray_workcell.pick_place:main",
    ]},
)
