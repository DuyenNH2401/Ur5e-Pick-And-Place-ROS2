"""Read object positions and operate Gazebo's built-in grasp joints."""
from functools import partial
import math
import threading

from gz.msgs.empty_pb2 import Empty
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.stringmsg_pb2 import StringMsg
from gz.transport import Node as TransportNode
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .moveit_client import MoveItClient


class Gazebo:
    def __init__(self, node, world, object_ids):
        self.transport = TransportNode()
        self.condition = threading.Condition()
        self.poses = {}
        self.states = {}
        self.commands = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.transport.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", self._poses)
        for name in object_ids:
            topic = f"/workcell/grasp/{name}"
            self.transport.subscribe(StringMsg, topic + "/state", partial(self._state, name))
            self.commands[name] = {
                command: self.transport.advertise(topic + "/" + command, Empty)
                for command in ("attach", "detach")}

    def _poses(self, message):
        with self.condition:
            for pose in message.pose:
                if pose.name in self.commands:
                    p = pose.position
                    self.poses[pose.name] = [p.x, p.y, p.z]
            self.condition.notify_all()

    def _state(self, name, message):
        with self.condition:
            self.states[name] = message.data
            self.condition.notify_all()

    def position(self, name):
        with self.condition:
            return list(self.poses[name])

    def command(self, name, command):
        publisher = self.commands[name][command]
        MoveItClient.wait(publisher.has_connections, 30, f"Gazebo joint unavailable: {name}")
        expected = "attached" if command == "attach" else "detached"
        with self.condition:
            self.states.pop(name, None)
            publisher.publish(Empty())
            if not self.condition.wait_for(lambda: self.states.get(name) == expected, timeout=5):
                raise RuntimeError(f"Gazebo did not acknowledge {command}: {name}")

    def initialize(self):
        with self.condition:
            if not self.condition.wait_for(lambda: self.commands.keys() <= self.poses.keys(), timeout=60):
                raise RuntimeError("No Gazebo object positions")
        # Jetty creates these joints attached. Release every object before motion.
        for name in self.commands:
            self.command(name, "detach")

    def attach(self, name):
        transform = self.buffer.lookup_transform("world", "tcp", Time())
        p = transform.transform.translation
        if math.dist([p.x, p.y, p.z], self.position(name)) > 0.025:
            raise RuntimeError(f"Gripper is too far from {name}")
        self.command(name, "attach")

    def detach(self, name):
        self.command(name, "detach")

    def close(self):
        for topic in self.transport.subscribed_topics():
            self.transport.unsubscribe(topic)
