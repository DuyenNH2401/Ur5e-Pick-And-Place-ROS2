"""Launch Gazebo, controllers, MoveIt and the Python pick/place task."""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tray_workcell.launch_config import prepare


def setup(context):
    camera = LaunchConfiguration("camera").perform(context) == "true"
    share, runtime, config, robot = prepare(with_camera=camera)
    gui = LaunchConfiguration("gui").perform(context) == "true"
    rviz = LaunchConfiguration("rviz").perform(context) == "true"
    result_file = LaunchConfiguration("result_file").perform(context)

    def load(name):
        return yaml.safe_load((share / "config" / name).read_text())

    moveit = {**robot, **load("moveit_controllers.yaml"),
              "planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl",
              "ompl": load("ompl_planning.yaml"),
              "publish_robot_description_semantic": True,
              "publish_planning_scene": True, "publish_geometry_updates": True,
              "publish_state_updates": True, "publish_transforms_updates": True}
    gazebo = ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "2", *([] if gui else ["-s"]),
                                 str(runtime / "world.sdf")], output="screen")
    spawn = Node(package="ros_gz_sim", executable="create", arguments=[
        "-world", config["world"], "-file", str(runtime / "robot.urdf"), "-name", "ur5e"], output="screen")
    controllers = Node(package="controller_manager", executable="spawner", arguments=[
        "joint_state_broadcaster", "arm_controller", "gripper_controller",
        "--controller-manager-timeout", "120", "--switch-timeout", "60",
        "--param-file", str(runtime / "controllers.yaml")], output="screen")
    task = Node(package="tray_workcell", executable="pick_place", parameters=[{
        "layout_file": str(runtime / "task.yaml"), "result_file": result_file,
        "use_sim_time": True}], output="screen")

    def after_spawn(event, _context):
        return [controllers] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason="Robot spawn failed"))]

    def after_controllers(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Controller setup failed"))]
        actions = [Node(package="moveit_ros_move_group", executable="move_group",
                        parameters=[moveit], output="screen")]
        if rviz:
            actions.append(Node(package="rviz2", executable="rviz2",
                                arguments=["-d", str(share / "rviz/workcell.rviz")],
                                parameters=[moveit], output="screen"))
        return [*actions, task]

    def after_task(event, _context):
        if event.returncode != 0:
            raise RuntimeError("Pick/place task failed; see its error above")
        return [] if gui or rviz else [EmitEvent(event=Shutdown())]

    resource_paths = [str(Path(get_package_share_directory(pkg)).parent)
                      for pkg in ("ur_description", "robotiq_description")]
    resource_paths.append(os.environ.get("GZ_SIM_RESOURCE_PATH", ""))
    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_paths)),
        SetEnvironmentVariable("GZ_LOG_PATH", str(runtime / "gz-log")),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[robot], output="screen"),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=controllers, on_exit=after_controllers)),
        RegisterEventHandler(OnProcessExit(target_action=task, on_exit=after_task)),
        RegisterEventHandler(OnProcessExit(target_action=gazebo, on_exit=[EmitEvent(event=Shutdown())])),
        gazebo,
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen"),
        spawn,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("camera", default_value="false"),
        DeclareLaunchArgument("result_file", default_value="/tmp/tray-workcell-result.yaml"),
        OpaqueFunction(function=setup),
    ])
