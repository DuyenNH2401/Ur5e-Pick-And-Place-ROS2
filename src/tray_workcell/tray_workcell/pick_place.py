"""Pick the blocks one by one and place them in the tray."""
import math
from pathlib import Path
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import yaml

from .gazebo_grasp import Gazebo
from .moveit_client import MoveItClient, collision_box


class PickPlace:
    def __init__(self, node):
        self.node = node
        self.config = yaml.safe_load(Path(node.get_parameter("layout_file").value).read_text())
        self.motion = self.config["motion"]
        self.objects = self.config["placements"]
        self.robot = MoveItClient(node, self.motion)
        self.gazebo = Gazebo(node, self.config["world"], [obj["id"] for obj in self.objects])
        self.result_file = Path(node.get_parameter("result_file").value)
        self.result = {"status": "running", "completed": 0, "expected": len(self.objects)}

    def setup_scene(self):
        boxes = [collision_box(box["id"], box["xyz"], box["size"])
                 for box in self.config["obstacles"]]
        boxes += [collision_box(obj["id"], self.gazebo.position(obj["id"]),
                                [self.config["object_size"]] * 3) for obj in self.objects]
        self.robot.scene(boxes=boxes)

    def pick_place(self, obj):
        name, target = obj["id"], obj["target"]
        source = self.gazebo.position(name)
        pick = [source[0], source[1], source[2] + self.motion["grasp_z_offset"]]
        approach = [pick[0], pick[1], pick[2] + self.motion["approach"]]
        lift = [pick[0], pick[1], self.motion["carry_height"]]
        above_tray = [target[0], target[1], self.motion["carry_height"]]
        place = [target[0], target[1], target[2] + self.motion["grasp_z_offset"]
                 + self.motion["release_clearance"]]
        # Robotiq linkage angle corresponding to the block width.
        close_angle = math.asin((0.085 - self.config["object_size"]) / (2 * 0.05715))
        self.node.get_logger().info(f"Pick {name} -> tray slot {obj['slot']}")

        self.robot.command_gripper(0.0)
        self.robot.move_pose(approach)
        self.robot.allow_touch(name, True)
        self.robot.move_pose(pick, cartesian=True)
        self.robot.command_gripper(close_angle)
        self.gazebo.attach(name)
        self.robot.attach(name)

        self.robot.move_pose(lift, cartesian=True)
        self.robot.move_pose(above_tray)
        self.robot.move_pose(place, cartesian=True)
        self.robot.command_gripper(0.0)
        self.gazebo.detach(name)
        self.robot.attach(name, remove=True)

        self.robot.scene(boxes=[collision_box(name, self.gazebo.position(name),
                                              [self.config["object_size"]] * 3)])
        self.robot.move_pose(above_tray, cartesian=True)
        self.robot.allow_touch(name, False)
        self.result["completed"] += 1

    def verify(self):
        def all_in_tray():
            for obj in self.objects:
                actual, target = self.gazebo.position(obj["id"]), obj["target"]
                if any(abs(actual[i] - target[i]) > self.motion["placement_tolerance"] for i in (0, 1)):
                    return False
                if abs(actual[2] - target[2]) > 0.008:
                    return False
            return True

        MoveItClient.wait(all_in_tray, 10, "Some blocks missed their tray slots")
        self.result.update(status="success", objects=[
            {"id": obj["id"], "actual": self.gazebo.position(obj["id"]), "target": obj["target"]}
            for obj in self.objects])
        self.node.get_logger().info(f"Verified {len(self.objects)} blocks in the tray")

    def run(self):
        self.gazebo.initialize()
        self.robot.wait_ready()
        self.setup_scene()
        for obj in self.objects:
            self.pick_place(obj)
        self.robot.home(self.config["robot"]["home"])
        self.verify()

    def save_result(self):
        self.result_file.parent.mkdir(parents=True, exist_ok=True)
        self.result_file.write_text(yaml.safe_dump(self.result, sort_keys=False))


def main(args=None):
    rclpy.init(args=args)
    node = Node("tray_pick_place", automatically_declare_parameters_from_overrides=True)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    task, code = None, 0
    try:
        task = PickPlace(node)
        task.run()
    except Exception as error:
        code = 1
        node.get_logger().error(str(error))
        if task:
            task.result.update(status="failed", error=str(error))
    finally:
        if task:
            task.gazebo.close()
        executor.shutdown()
        spin.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if task:
            task.save_result()
    return code
