"""Integration checks against the installed ROS descriptions after colcon build."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/tray_workcell"))
from tray_workcell.launch_config import prepare


def test_robot_contains_real_gripper_and_correct_control_interfaces():
    _, _, _, params = prepare()
    root = ET.fromstring(params["robot_description"])
    assert root.find("link[@name='tcp']") is not None
    assert root.find("link[@name='robotiq_85_base_link']") is not None
    control = root.find("ros2_control")
    commanded = {j.attrib["name"] for j in control.findall("joint") if j.find("command_interface") is not None}
    assert len(commanded) == 7
    for joint in root.findall("joint"):
        if joint.find("mimic") is not None:
            assert joint.attrib["name"] not in commanded
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib["filename"]
        if filename.startswith("package://"):
            from ament_index_python.packages import get_package_share_directory
            package, relative = filename[10:].split("/", 1)
            assert (Path(get_package_share_directory(package)) / relative).is_file()
        else:
            assert Path(filename.removeprefix("file://")).is_file()


def test_assisted_uses_native_joints_for_exactly_the_configured_objects():
    _, _, config, params = prepare()
    root = ET.fromstring(params["robot_description"])
    plugins = root.findall("gazebo/plugin[@name='gz::sim::systems::DetachableJoint']")
    assert {p.findtext("child_model") for p in plugins} == {o["id"] for o in config["objects"]}
    assert all(p.attrib["filename"] == "gz-sim-detachable-joint-system" for p in plugins)
    assert not any("libtray_grasp" in p.attrib["filename"] for p in root.findall("gazebo/plugin"))
