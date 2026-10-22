#!/usr/bin/env python3
"""Run the Gazebo pick-and-place simulation with camera and record demo MP4 & GIF."""
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import cv2
import numpy as np
import yaml

# Set up shared partition BEFORE initializing Gazebo TransportNode
partition = "tray-" + uuid.uuid4().hex
os.environ["GZ_PARTITION"] = partition
os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "87")
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

from gz.msgs.image_pb2 import Image
from gz.transport import Node as TransportNode


def main():
    root = Path(__file__).resolve().parents[1]
    assets_dir = root / "assets"
    assets_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = root / ".runtime" / f"{timestamp}-demo"
    run_dir.mkdir(parents=True, exist_ok=True)
    result_file = run_dir / "result.yaml"

    os.environ["ROS_LOG_DIR"] = str(run_dir / "ros-log")

    raw_video_path = run_dir / "raw_recording.mp4"
    frame_width = 960
    frame_height = 600
    fps = 15

    # Pipe directly to ffmpeg for rock-solid encoding
    cmd_encode = [
        "ffmpeg", "-y", "-f", "rawvideo", "-vcodec", "rawvideo",
        "-s", f"{frame_width}x{frame_height}", "-pix_fmt", "bgr24", "-r", str(fps),
        "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18",
        str(raw_video_path)
    ]
    ffmpeg_proc = subprocess.Popen(cmd_encode, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)

    recording = True
    frame_count = 0
    lock = threading.Lock()

    def image_callback(msg):
        nonlocal frame_count
        if not recording:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if (msg.width, msg.height) != (frame_width, frame_height):
            bgr = cv2.resize(bgr, (frame_width, frame_height), interpolation=cv2.INTER_AREA)
        with lock:
            if ffmpeg_proc.stdin and not ffmpeg_proc.stdin.closed:
                try:
                    ffmpeg_proc.stdin.write(bgr.tobytes())
                    frame_count += 1
                    if frame_count % 30 == 0:
                        print(f"[Recorder] Captured {frame_count} frames...", flush=True)
                except (BrokenPipeError, OSError):
                    pass

    node = TransportNode()
    node.subscribe(Image, "/demo_camera/image", image_callback)

    command = [
        "ros2", "launch", "tray_workcell", "workcell.launch.py",
        "gui:=false", "rviz:=false", "camera:=true",
        f"result_file:={result_file}"
    ]

    print(f"[Recorder] Partition: {partition}")
    print(f"[Recorder] Launching simulation with camera...")
    print(f"[Recorder] Run directory: {run_dir}")

    launch_log = run_dir / "launch.log"
    with launch_log.open("w") as log:
        process = subprocess.Popen(
            command, cwd=root, env=os.environ, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
        )
        try:
            while process.poll() is None:
                time.sleep(2)
                if result_file.exists():
                    try:
                        data = yaml.safe_load(result_file.read_text()) or {}
                        if data.get("status") == "success" and data.get("completed", 0) == data.get("expected", -1):
                            print("[Recorder] Task completed successfully! Capturing extra frames...", flush=True)
                            time.sleep(3)
                            break
                    except Exception:
                        pass
        except KeyboardInterrupt:
            print("[Recorder] Stopped by user.")
        finally:
            recording = False
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()

    with lock:
        if ffmpeg_proc.stdin and not ffmpeg_proc.stdin.closed:
            ffmpeg_proc.stdin.close()
    ffmpeg_proc.wait()

    print(f"[Recorder] Total raw frames recorded: {frame_count}")

    if frame_count == 0 or not raw_video_path.exists():
        print("[Recorder] Error: No frames were captured.")
        return 1

    demo_mp4 = assets_dir / "demo.mp4"
    demo_gif = assets_dir / "demo.gif"

    print(f"[Recorder] Generating speed-up MP4: {demo_mp4}...")
    cmd_mp4 = [
        "ffmpeg", "-y", "-i", str(raw_video_path),
        "-vf", "setpts=0.4*PTS",
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
        str(demo_mp4)
    ]
    subprocess.run(cmd_mp4, check=True)

    print(f"[Recorder] Generating optimized GIF: {demo_gif}...")
    cmd_gif = [
        "ffmpeg", "-y", "-i", str(raw_video_path),
        "-vf", "setpts=0.4*PTS,fps=12,scale=720:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
        str(demo_gif)
    ]
    subprocess.run(cmd_gif, check=True)

    gif_size_mb = demo_gif.stat().st_size / (1024 * 1024)
    mp4_size_mb = demo_mp4.stat().st_size / (1024 * 1024)
    print("[Recorder] SUCCESS!")
    print(f"  -> MP4: {demo_mp4} ({mp4_size_mb:.2f} MB)")
    print(f"  -> GIF: {demo_gif} ({gif_size_mb:.2f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
