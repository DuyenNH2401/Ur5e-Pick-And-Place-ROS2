"""One geometry source for Gazebo and the MoveIt planning scene (SI units)."""
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


def load_layout(path):
    return yaml.safe_load(Path(path).read_text())


def placements(config):
    tray = config["tray"]
    result = []
    for obj in config["objects"]:
        row, column = divmod(obj["slot"], tray["columns"])
        target = [tray["center"][0] + (column - (tray["columns"] - 1) / 2) * tray["cell_size"][0],
                  tray["center"][1] + (row - (tray["rows"] - 1) / 2) * tray["cell_size"][1],
                  config["table"]["height"] + tray["floor"] + config["object_size"] / 2]
        result.append({**obj, "source": [*obj["xy"], config["table"]["height"] + config["object_size"] / 2],
                       "target": target})
    return result


def obstacles(config):
    table, tray = config["table"], config["tray"]
    center_x, center_y = tray["center"]
    width, depth = tray["columns"] * tray["cell_size"][0], tray["rows"] * tray["cell_size"][1]
    wall, floor = tray["wall"], tray["floor"]
    wall_height, table_height = tray["wall_height"], table["height"]
    wall_z = table_height + floor + wall_height / 2
    boxes = [{"id": "table_top", "xyz": [*table["center"], table_height - table["thickness"] / 2],
              "size": [*table["size"], table["thickness"]]},
             {"id": "tray_floor", "xyz": [center_x, center_y, table_height + floor / 2],
              "size": [width + wall, depth + wall, floor]}]
    for i, sign_x in enumerate((-1, 1)):
        for j, sign_y in enumerate((-1, 1)):
            boxes.append({"id": f"table_leg_{i}_{j}",
                          "xyz": [table["center"][0] + sign_x * (table["size"][0] / 2 - .05),
                                  table["center"][1] + sign_y * (table["size"][1] / 2 - .05),
                                  (table_height - table["thickness"]) / 2],
                          "size": [.045, .045, table_height - table["thickness"]]})
    for sign in (-1, 1):
        boxes.append({"id": f"tray_x_{sign}", "xyz": [center_x + sign * width / 2, center_y, wall_z],
                      "size": [wall, depth + wall, wall_height]})
        boxes.append({"id": f"tray_y_{sign}", "xyz": [center_x, center_y + sign * depth / 2, wall_z],
                      "size": [width + wall, wall, wall_height]})
    for column in range(1, tray["columns"]):
        boxes.append({"id": f"tray_column_{column}",
                      "xyz": [center_x - width / 2 + column * tray["cell_size"][0], center_y, wall_z],
                      "size": [wall, depth - wall, wall_height]})
    for row in range(1, tray["rows"]):
        boxes.append({"id": f"tray_row_{row}",
                      "xyz": [center_x, center_y - depth / 2 + row * tray["cell_size"][1], wall_z],
                      "size": [width - wall, wall, wall_height]})
    return boxes


def task_data(config):
    return {**config, "placements": placements(config), "obstacles": obstacles(config)}


def world_sdf(config):
    root = ET.Element("sdf", version="1.10")
    world = ET.SubElement(root, "world", name=config["world"])
    physics = ET.SubElement(world, "physics", name="physics", type="ignored")
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1.0"
    for system in ("Physics", "UserCommands", "SceneBroadcaster"):
        filename = {"Physics": "physics", "UserCommands": "user-commands", "SceneBroadcaster": "scene-broadcaster"}[system]
        ET.SubElement(world, "plugin", filename=f"gz-sim-{filename}-system", name=f"gz::sim::systems::{system}")
    light = ET.SubElement(world, "light", name="sun", type="directional")
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.9 0.9 0.9 1"
    ET.SubElement(light, "direction").text = "-0.5 -0.5 -1"

    def box(link, name, xyz, size, color):
        for kind in ("visual", "collision"):
            part = ET.SubElement(link, kind, name=name + "_visual" if kind == "visual" else name)
            ET.SubElement(part, "pose").text = " ".join(map(str, [*xyz, 0, 0, 0]))
            geometry = ET.SubElement(part, "geometry")
            ET.SubElement(ET.SubElement(geometry, "box"), "size").text = " ".join(map(str, size))
            if kind == "visual":
                material = ET.SubElement(part, "material")
                for prop in ("ambient", "diffuse"):
                    ET.SubElement(material, prop).text = " ".join(map(str, color))

    def model(name, static=True):
        node = ET.SubElement(world, "model", name=name)
        ET.SubElement(node, "static").text = str(static).lower()
        return node, ET.SubElement(node, "link", name="link")

    _, ground = model("ground")
    box(ground, "ground", [0, 0, -0.05], [5, 5, 0.1], [.65, .65, .65, 1])
    _, table = model("table")
    _, tray = model("tray")
    for obstacle in obstacles(config):
        box(table if obstacle["id"].startswith("table_") else tray,
            "floor" if obstacle["id"] == "tray_floor" else obstacle["id"], obstacle["xyz"], obstacle["size"],
            [.45, .30, .18, 1] if obstacle["id"] == "table_top" else [.35, .4, .5, 1])
    for obj in placements(config):
        node, link = model(obj["id"], False)
        ET.SubElement(node, "pose").text = " ".join(map(str, [*obj["source"], 0, 0, 0]))
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass").text = str(config["object_mass"])
        inertia = ET.SubElement(inertial, "inertia")
        for axis in ("ixx", "iyy", "izz"):
            ET.SubElement(inertia, axis).text = str(config["object_mass"] * config["object_size"] ** 2 / 6)
        box(link, "body", [0, 0, 0], [config["object_size"]] * 3, obj["color"])
    ET.indent(root)
    return ET.tostring(root, encoding="unicode")
