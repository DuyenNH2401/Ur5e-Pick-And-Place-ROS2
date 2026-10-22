"""Prepare the robot, world and MoveIt configuration for launch."""
from itertools import combinations
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

from .layout import load_layout, task_data, world_sdf


def prepare(with_camera: bool = False):
    share = Path(get_package_share_directory("tray_workcell"))
    config = load_layout(share / "config/workcell.yaml")
    runtime = Path(tempfile.mkdtemp(prefix="tray-workcell-"))
    control_config = yaml.safe_load((share / "config/controllers.yaml").read_text())
    for settings in control_config.values():
        settings["ros__parameters"]["use_sim_time"] = True
    controllers = str(runtime / "controllers.yaml")
    Path(controllers).write_text(yaml.safe_dump(control_config))
    xml = xacro.process_file(str(share / "urdf/workcell.urdf.xacro"), mappings={
        "object_ids": " ".join(obj["id"] for obj in config["objects"]),
        "base_xyz": " ".join(map(str, config["robot"]["base"])),
        "controllers_file": controllers}).toxml()
    urdf = ET.fromstring(xml)
    srdf = ET.parse(share / "config/workcell.srdf").getroot()
    disabled = set()

    def disable(a, b, reason):
        pair = tuple(sorted((a, b)))
        if pair not in disabled:
            ET.SubElement(srdf, "disable_collisions", link1=a, link2=b, reason=reason)
            disabled.add(pair)

    for item in srdf.findall("disable_collisions"):
        disabled.add(tuple(sorted((item.attrib["link1"], item.attrib["link2"]))))
    for joint in urdf.findall("joint"):
        disable(joint.find("parent").attrib["link"], joint.find("child").attrib["link"], "Adjacent")
    # Coupled Robotiq linkage meshes intersect at their pivots by design. All
    # arm-to-gripper and gripper-to-world collisions remain enabled.
    gripper_links = [link.attrib["name"] for link in urdf.findall("link")
                     if link.attrib["name"].startswith("robotiq_")]
    for a, b in combinations(gripper_links, 2):
        disable(a, b, "Coupled gripper linkage")
    # The tool frame and gripper base are rigidly mounted on wrist_3.
    disable("wrist_3_link", "robotiq_85_base_link", "Rigid mounting")
    limits = {}
    for joint in urdf.findall("joint"):
        if joint.attrib["type"] in ("fixed", "floating"):
            continue
        limit = joint.find("limit")
        limits[joint.attrib["name"]] = {
            "has_velocity_limits": True, "max_velocity": float(limit.attrib["velocity"]),
            "has_acceleration_limits": True, "max_acceleration": 1.0}
    def yaml_load(filename):
        return yaml.safe_load((share / "config" / filename).read_text())

    params = {
        "robot_description": xml,
        "robot_description_semantic": ET.tostring(srdf, encoding="unicode"),
        "robot_description_kinematics": yaml_load("kinematics.yaml"),
        "robot_description_planning": {"joint_limits": limits,
                                       "default_velocity_scaling_factor": config["motion"]["velocity_scale"],
                                       "default_acceleration_scaling_factor": config["motion"]["acceleration_scale"]},
        "use_sim_time": True}
    (runtime / "robot.urdf").write_text(xml)
    sdf_content = world_sdf(config)
    if with_camera:
        root = ET.fromstring(sdf_content)
        world = root.find("world")
        plugin = ET.SubElement(world, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors")
        ET.SubElement(plugin, "render_engine").text = "ogre2"

        cam_model = ET.SubElement(world, "model", name="demo_camera")
        ET.SubElement(cam_model, "static").text = "true"
        ET.SubElement(cam_model, "pose").text = "1.45 -0.15 1.50 0 0.48 2.98"
        cam_link = ET.SubElement(cam_model, "link", name="camera_link")
        sensor = ET.SubElement(cam_link, "sensor", name="camera", type="camera")
        ET.SubElement(sensor, "always_on").text = "1"
        ET.SubElement(sensor, "update_rate").text = "15"
        ET.SubElement(sensor, "topic").text = "demo_camera/image"
        cam = ET.SubElement(sensor, "camera")
        ET.SubElement(cam, "horizontal_fov").text = "1.25"
        img = ET.SubElement(cam, "image")
        ET.SubElement(img, "width").text = "960"
        ET.SubElement(img, "height").text = "600"
        clip = ET.SubElement(cam, "clip")
        ET.SubElement(clip, "near").text = "0.1"
        ET.SubElement(clip, "far").text = "10.0"
        sdf_content = ET.tostring(root, encoding="unicode")
    (runtime / "world.sdf").write_text(sdf_content)
    (runtime / "task.yaml").write_text(yaml.safe_dump(task_data(config)))
    return share, runtime, config, params
