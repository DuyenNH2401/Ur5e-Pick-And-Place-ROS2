import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/tray_workcell"))
from tray_workcell.layout import load_layout, obstacles, placements, world_sdf


def test_six_distinct_slots_fit_inside_tray():
    config = load_layout(ROOT / "src/tray_workcell/config/workcell.yaml")
    targets = placements(config)
    assert len(targets) == 6
    assert len({tuple(t["target"]) for t in targets}) == 6
    for target in targets:
        assert target["target"][2] > config["table"]["height"]


def test_world_contains_hollow_tray_and_dynamic_objects():
    import xml.etree.ElementTree as ET
    config = load_layout(ROOT / "src/tray_workcell/config/workcell.yaml")
    root = ET.fromstring(world_sdf(config))
    world = root.find("world")
    tray = world.find("model[@name='tray']/link")
    assert len(tray.findall("collision")) == 8  # floor + 4 walls + 3 dividers
    floor_thickness = float(tray.findtext("collision[@name='floor']/geometry/box/size").split()[2])
    assert floor_thickness == config["tray"]["floor"]
    for obj in config["objects"]:
        model = world.find(f"model[@name='{obj['id']}']")
        assert model.findtext("static") == "false"
        assert model.find("link/inertial/mass") is not None


def test_sdf_names_unique_and_collision_geometry_matches_moveit():
    import xml.etree.ElementTree as ET
    config = load_layout(ROOT / "src/tray_workcell/config/workcell.yaml")
    world = ET.fromstring(world_sdf(config)).find("world")
    for link in world.findall("model/link"):
        names = [x.attrib["name"] for x in link if "name" in x.attrib]
        assert len(names) == len(set(names))
    for ob in obstacles(config):
        name = "floor" if ob["id"] == "tray_floor" else ob["id"]
        model = "table" if ob["id"].startswith("table_") else "tray"
        collision = world.find(f"model[@name='{model}']/link/collision[@name='{name}']")
        assert [float(x) for x in collision.findtext("geometry/box/size").split()] == ob["size"]
        assert [float(x) for x in collision.findtext("pose").split()[:3]] == ob["xyz"]
