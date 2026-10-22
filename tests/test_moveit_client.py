"""Request semantics and failure gates, independent of a running ROS graph."""
import sys
from pathlib import Path

import pytest
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/tray_workcell"))
from tray_workcell.moveit_client import (
    downward_pose, pose_constraints, cartesian_request, validate_cartesian, set_allowed_touch, MoveItClient,
)


def test_pose_is_unit_quaternion_and_tcp_points_down():
    p = downward_pose([0.3, -0.2, 0.8])
    q = p.orientation
    assert q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w == 1.0
    assert 1 - 2 * (q.x * q.x + q.y * q.y) == -1
    assert q.w == 0


def test_pose_goal_constrains_both_position_and_orientation():
    c = pose_constraints([0.3, -0.2, 0.8])
    assert c.position_constraints[0].header.frame_id == "world"
    assert c.position_constraints[0].link_name == "tcp"
    assert c.position_constraints[0].constraint_region.primitive_poses[0].position.z == 0.8
    assert c.orientation_constraints[0].orientation.w == 0
    assert c.orientation_constraints[0].weight == 1.0


def test_cartesian_checks_collisions_and_scales_speed():
    r = cartesian_request([0.3, -0.2, 0.8], 0.005, 0.25, 0.2)
    assert r.avoid_collisions is True
    assert r.start_state.is_diff is True
    assert r.group_name == "ur_manipulator" and r.link_name == "tcp"
    assert r.max_step == 0.005 and r.max_velocity_scaling_factor == 0.25
    assert r.max_acceleration_scaling_factor == 0.2


def test_partial_or_failed_cartesian_paths_are_never_executed():
    from moveit_msgs.srv import GetCartesianPath
    response = GetCartesianPath.Response()
    response.error_code.val = MoveItErrorCodes.SUCCESS
    response.fraction = 0.8
    response.solution.joint_trajectory.points = [JointTrajectoryPoint()]
    with pytest.raises(RuntimeError, match="incomplete"):
        validate_cartesian(response)
    response.fraction = 1.0
    response.error_code.val = MoveItErrorCodes.PLANNING_FAILED
    with pytest.raises(RuntimeError):
        validate_cartesian(response)
    response.error_code.val = MoveItErrorCodes.SUCCESS
    assert validate_cartesian(response) is response.solution
    response.fraction = float("nan")
    with pytest.raises(RuntimeError, match="incomplete"):
        validate_cartesian(response)


def test_allowed_touch_preserves_other_collision_rules_and_can_revoke():
    from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
    matrix = AllowedCollisionMatrix(entry_names=["arm", "wall"],
        entry_values=[AllowedCollisionEntry(enabled=[False, True]), AllowedCollisionEntry(enabled=[True, False])])
    set_allowed_touch(matrix, "block_0", ["finger"], True)
    ids = matrix.entry_names
    assert matrix.entry_values[ids.index("arm")].enabled[ids.index("wall")] is True
    assert matrix.entry_values[ids.index("block_0")].enabled[ids.index("finger")] is True
    assert matrix.entry_values[ids.index("block_0")].enabled[ids.index("arm")] is False
    set_allowed_touch(matrix, "block_0", ["finger"], False)
    assert matrix.entry_values[ids.index("finger")].enabled[ids.index("block_0")] is False


def test_planning_request_carries_measured_start_joints():
    from sensor_msgs.msg import JointState
    client = object.__new__(MoveItClient)
    client.motion = {"velocity_scale": .25, "acceleration_scale": .2}
    client.joints = JointState(name=["shoulder_lift_joint"], position=[-1.57])
    goal = client._goal()
    assert goal.request.start_state.joint_state.name == ["shoulder_lift_joint"]
    assert list(goal.request.start_state.joint_state.position) == [-1.57]
