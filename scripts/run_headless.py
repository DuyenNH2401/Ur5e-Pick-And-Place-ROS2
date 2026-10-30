#!/usr/bin/env python3
"""Run the six-block Gazebo demo without windows and save its result."""
import os
import signal
import subprocess
import uuid
from datetime import datetime
from pathlib import Path

import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    run = root / ".runtime" / (datetime.now().strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6])
    run.mkdir(parents=True)
    result = run / "result.yaml"
    env = {**os.environ, "ROS_LOG_DIR": str(run / "ros-log"),
           "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "87"),
           "GZ_PARTITION": "tray-" + uuid.uuid4().hex,
           "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"}
    command = ["ros2", "launch", "tray_workcell", "workcell.launch.py",
               "gui:=false", "rviz:=false", f"result_file:={result}"]
    print(f"Log and result: {run}", flush=True)
    with (run / "launch.log").open("w") as log:
        process = subprocess.Popen(command, cwd=root, env=env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            process.wait(timeout=900)
        except subprocess.TimeoutExpired:
            pass
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    data = yaml.safe_load(result.read_text()) if result.exists() else {}
    success = (process.returncode == 0 and data.get("status") == "success"
               and data.get("completed", 0) == data.get("expected", -1))
    print(yaml.safe_dump(data, sort_keys=False))
    print("PASS" if success else f"FAILED: see {run / 'launch.log'}")
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
