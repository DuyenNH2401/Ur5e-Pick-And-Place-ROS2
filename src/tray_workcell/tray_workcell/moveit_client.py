"""MoveIt 2 via public ROS actions/services; all application logic is Python."""
import copy
import math
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import ParallelGripperCommand, FollowJointTrajectory
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry, AttachedCollisionObject, CollisionObject, Constraints,
    JointConstraint, MoveItErrorCodes, OrientationConstraint, PlanningScene,
    PlanningSceneComponents, PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TOUCH_LINKS = [f"robotiq_85_{side}_{part}_link" for side in ("left", "right")
               for part in ("finger_tip", "finger", "inner_knuckle")]


def downward_pose(xyz):
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, xyz)
    result.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    return result


def pose_constraints(xyz):
    pose = downward_pose(xyz)
    position = PositionConstraint(link_name="tcp", weight=1.0)
    position.header.frame_id = "world"
    position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])]
    region = copy.deepcopy(pose)
    region.orientation = Quaternion(w=1.0)
    position.constraint_region.primitive_poses = [region]
    orientation = OrientationConstraint(link_name="tcp", orientation=pose.orientation,
        absolute_x_axis_tolerance=0.02, absolute_y_axis_tolerance=0.02,
        absolute_z_axis_tolerance=0.02, weight=1.0)
    orientation.header.frame_id = "world"
    return Constraints(position_constraints=[position], orientation_constraints=[orientation])


def cartesian_request(xyz, step, velocity, acceleration):
    request = GetCartesianPath.Request()
    request.header.frame_id = "world"
    request.start_state.is_diff = True
    request.group_name = "ur_manipulator"
    request.link_name = "tcp"
    request.waypoints = [downward_pose(xyz)]
    request.max_step = float(step)
    request.avoid_collisions = True
    request.max_velocity_scaling_factor = float(velocity)
    request.max_acceleration_scaling_factor = float(acceleration)
    return request


def validate_cartesian(response):
    if response.error_code.val != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(f"Cartesian planning failed: {response.error_code.val}")
    if not math.isfinite(response.fraction) or not 0.999 <= response.fraction <= 1.000001 or \
            not response.solution.joint_trajectory.points:
        raise RuntimeError(f"incomplete Cartesian path: fraction={response.fraction:.6f}")
    return response.solution


def set_allowed_touch(matrix, object_id, links, allowed):
    """Extend a symmetric ACM without dropping any pre-existing rules."""
    for name in [object_id, *links]:
        if name not in matrix.entry_names:
            for row in matrix.entry_values:
                row.enabled.append(False)
            matrix.entry_names.append(name)
            matrix.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names)))
    i = matrix.entry_names.index(object_id)
    for link in links:
        j = matrix.entry_names.index(link)
        matrix.entry_values[i].enabled[j] = allowed
        matrix.entry_values[j].enabled[i] = allowed


def collision_box(object_id, xyz, size):
    box = CollisionObject(id=object_id, operation=CollisionObject.ADD)
    box.header.frame_id = "world"
    box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, size)))]
    p = downward_pose(xyz)
    p.orientation = Quaternion(w=1.0)
    box.primitive_poses = [p]
    return box


class MoveItClient:
    """Synchronous facade; a background executor must keep spinning the node."""

    def __init__(self, node, motion):
        self.node, self.motion = node, motion
        self.joints = None
        self.subscription = node.create_subscription(JointState, "/joint_states", self._joints, 10)
        self.move = ActionClient(node, MoveGroup, "/move_action")
        self.execute = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
        self.gripper = ActionClient(node, ParallelGripperCommand, "/gripper_controller/gripper_cmd")
        self.arm_controller = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.cartesian = node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.get_scene = node.create_client(GetPlanningScene, "/get_planning_scene")
        self.apply_scene = node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def _joints(self, msg):
        self.joints = msg

    def wait_ready(self):
        for client in (self.move, self.execute, self.gripper, self.arm_controller):
            if not client.wait_for_server(timeout_sec=90.0):
                raise RuntimeError("MoveIt/controller action server unavailable")
        for client in (self.cartesian, self.get_scene, self.apply_scene):
            if not client.wait_for_service(timeout_sec=30.0):
                raise RuntimeError(f"Service unavailable: {client.srv_name}")
        self.wait(lambda: self.joints is not None and set(ARM_JOINTS) <= set(self.joints.name), 30.0,
                  "No complete joint state")
        # The planning monitor is a separate subscriber. Its services may exist
        # before it has received the measured home state. Poll the actual state.
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ROBOT_STATE
            scene = self.await_future(self.get_scene.call_async(request), 5, "Scene readiness timeout").scene
            actual = dict(zip(scene.robot_state.joint_state.name, scene.robot_state.joint_state.position))
            measured = dict(zip(self.joints.name, self.joints.position))
            if all(name in actual and abs(actual[name] - measured[name]) < .01 for name in ARM_JOINTS):
                return
            time.sleep(.05)
        raise RuntimeError("MoveIt planning monitor did not receive measured joints")

    @staticmethod
    def wait(predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        raise RuntimeError(message)

    def await_future(self, future, timeout, message):
        self.wait(future.done, timeout, message)
        return future.result()

    def action(self, client, goal, timeout=120.0):
        handle = self.await_future(client.send_goal_async(goal), 10.0, "Action acceptance timeout")
        if not handle.accepted:
            raise RuntimeError("Action goal rejected")
        try:
            result = self.await_future(handle.get_result_async(), timeout, "Action execution timeout")
        except Exception:
            handle.cancel_goal_async()
            raise
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            error = getattr(getattr(result.result, "error_code", None), "val", "unknown")
            raise RuntimeError(f"Action failed: status={result.status}, error={error}")
        if hasattr(result.result, "error_code") and result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt failed: error={result.result.error_code.val}")
        return result.result

    def scene(self, *, boxes=(), attached=(), matrix=None):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = list(boxes)
        scene.robot_state.attached_collision_objects = list(attached)
        if matrix is not None:
            scene.allowed_collision_matrix = matrix
        response = self.await_future(self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)),
                                     10.0, "Planning scene update timeout")
        if not response.success:
            raise RuntimeError("Planning scene update failed")

    def allow_touch(self, object_id, allowed):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        response = self.await_future(self.get_scene.call_async(request), 10.0, "ACM read timeout")
        matrix = response.scene.allowed_collision_matrix
        set_allowed_touch(matrix, object_id, TOUCH_LINKS, allowed)
        self.scene(matrix=matrix)

    def attach(self, object_id, remove=False):
        attached = AttachedCollisionObject(link_name="tcp", touch_links=TOUCH_LINKS)
        attached.object.id = object_id
        attached.object.operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
        self.scene(attached=[attached])

    def _goal(self):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = "ur_manipulator"
        request.pipeline_id = "ompl"
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = self.motion["velocity_scale"]
        request.max_acceleration_scaling_factor = self.motion["acceleration_scale"]
        request.start_state.is_diff = True
        request.start_state.joint_state = self.measured_joints()
        request.workspace_parameters.header.frame_id = "world"
        request.workspace_parameters.min_corner.x = -1.5
        request.workspace_parameters.min_corner.y = -1.5
        request.workspace_parameters.min_corner.z = 0.0
        request.workspace_parameters.max_corner.x = 1.5
        request.workspace_parameters.max_corner.y = 1.5
        request.workspace_parameters.max_corner.z = 2.0
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def measured_joints(self):
        return copy.deepcopy(self.joints)

    def execute_trajectory(self, trajectory):
        self.action(self.execute, ExecuteTrajectory.Goal(trajectory=trajectory, controller_names=["arm_controller"]))

    def move_pose(self, xyz, cartesian=False):
        if cartesian:
            request = cartesian_request(
                xyz, self.motion["cartesian_step"], self.motion["velocity_scale"],
                self.motion["acceleration_scale"])
            request.start_state.joint_state = self.measured_joints()
            response = self.await_future(self.cartesian.call_async(request), 30.0, "Cartesian planning timeout")
            trajectory = validate_cartesian(response)
        else:
            goal = self._goal()
            goal.request.goal_constraints = [pose_constraints(xyz)]
            trajectory = self.action(self.move, goal, 40.0).planned_trajectory
        self.execute_trajectory(trajectory)

    def home(self, values):
        goal = self._goal()
        goal.request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=float(value), tolerance_above=0.001,
                            tolerance_below=0.001, weight=1.0) for name, value in zip(ARM_JOINTS, values)])]
        self.execute_trajectory(self.action(self.move, goal, 40.0).planned_trajectory)

    def command_gripper(self, position):
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ["robotiq_85_left_knuckle_joint"]
        goal.command.position = [float(position)]
        result = self.action(self.gripper, goal, 20.0)
        if not (result.reached_goal or (position > 0 and result.stalled)):
            raise RuntimeError("Gripper failed to reach target or establish stalled grasp")
