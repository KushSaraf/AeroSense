"""`ros2 run aero_sense_bringup system_check` — verify the Aero Sense stack.

Static checks always run. Live checks (ROS topics, MAVLink heartbeat) run only when a
simulation is up and report SKIP otherwise, so the command is useful before and after
launch. Exit code is non-zero if anything FAILs.
"""
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"
MIN_FREE_DISK_GB = 2.0
TOPIC_DISCOVERY_S = 2.0
HEARTBEAT_TIMEOUT_S = 3.0
#: MAVProxy's diagnostics output (simulation.launch.py); 14550 is left to the GCS.
MAVLINK_DIAGNOSTICS_URL = "udpin:127.0.0.1:14552"
UAV_WS = Path(os.environ.get("UAV_WS", Path.home() / "uav_ws"))
WORKSPACE_PACKAGES = ("aero_sense_interfaces", "aero_sense_bringup", "aero_sense_description",
                      "aero_sense_gazebo", "aero_sense_mission", "aero_sense_scenario_manager",
                      "aero_sense_perception", "aero_sense_visualization",
                      "aero_sense_bridge")
#: Topics that must exist while the simulation runs (grows as phases land).
LIVE_TOPICS = (
    "/clock",
    "/aero_sense/drone/pose", "/aero_sense/drone/status", "/aero_sense/drone/battery",
    "/aero_sense/gps/fix",
    "/aero_sense/camera/rgb/image_raw", "/aero_sense/camera/depth/image_raw",
    "/aero_sense/camera/thermal/image_raw", "/aero_sense/camera/rgb/camera_info",
    "/aero_sense/lidar/points", "/aero_sense/imu", "/aero_sense/baro",
    "/aero_sense/ground_truth/victims",
    "/aero_sense/victims", "/aero_sense/perception/detections",
)


def check_ros() -> tuple:
    distro = os.environ.get("ROS_DISTRO")
    return (PASS, f"ROS 2 {distro}") if distro else (FAIL, "ROS 2 not sourced")


def check_workspace() -> tuple:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    missing = []
    for pkg in WORKSPACE_PACKAGES:
        try:
            get_package_share_directory(pkg)
        except PackageNotFoundError:
            missing.append(pkg)
    return (FAIL, f"not found: {', '.join(missing)} (source install/setup.bash)") if missing \
        else (PASS, f"{len(WORKSPACE_PACKAGES)} packages")


def check_interfaces() -> tuple:
    try:
        from aero_sense_interfaces.msg import Alert, VictimArray  # noqa: F401
    except ImportError as exc:
        return FAIL, f"import failed: {exc}"
    return PASS, "messages import"


def check_gazebo() -> tuple:
    if not shutil.which("gz"):
        return FAIL, "gz not on PATH"
    out = subprocess.run(["gz", "sim", "--versions"], capture_output=True, text=True, timeout=20)
    return (PASS, f"Gazebo {out.stdout.strip()}") if out.returncode == 0 else (FAIL, out.stderr.strip())


def check_sitl() -> tuple:
    binary = UAV_WS / "src/ardupilot/build/sitl/bin/arducopter"
    return (PASS, str(binary)) if binary.exists() else (FAIL, f"missing {binary}")


def check_reference_assets() -> tuple:
    from aero_sense_bringup import worlds
    try:
        paths = worlds.reference_model_paths()
    except FileNotFoundError as exc:
        return FAIL, str(exc)
    return PASS, f"{sum(1 for p in paths for _ in p.iterdir())} models in {len(paths)} repos"


def check_disk() -> tuple:
    free_gb = shutil.disk_usage(Path.home()).free / 1e9
    status = PASS if free_gb >= MIN_FREE_DISK_GB else FAIL
    return status, f"{free_gb:.1f} GB free"


def check_topics() -> tuple:
    if not LIVE_TOPICS:
        return SKIP, "no live topics defined yet"
    import rclpy
    rclpy.init()
    node = rclpy.create_node("aero_sense_system_check")
    try:
        deadline = time.time() + TOPIC_DISCOVERY_S
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        present = {name for name, _ in node.get_topic_names_and_types()}
    finally:
        node.destroy_node()
        rclpy.shutdown()
    missing = [t for t in LIVE_TOPICS if t not in present]
    if len(missing) == len(LIVE_TOPICS):
        return SKIP, "simulation not running"
    return (FAIL, f"missing: {', '.join(missing)}") if missing else (PASS, f"{len(LIVE_TOPICS)} topics")


def check_mavlink() -> tuple:
    from pymavlink import mavutil
    try:
        conn = mavutil.mavlink_connection(MAVLINK_DIAGNOSTICS_URL, source_system=245)
    except OSError as exc:
        return FAIL, f"cannot listen on {MAVLINK_DIAGNOSTICS_URL}: {exc}"
    try:
        beat = conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
    finally:
        conn.close()
    return (PASS, f"heartbeat from system {beat.get_srcSystem()}") if beat else (SKIP, "no autopilot running")


CHECKS = (
    ("ROS 2", check_ros),
    ("Workspace", check_workspace),
    ("Interfaces", check_interfaces),
    ("Gazebo", check_gazebo),
    ("ArduPilot SITL", check_sitl),
    ("Reference assets", check_reference_assets),
    ("Disk", check_disk),
    ("ROS topics", check_topics),
    ("MAVLink", check_mavlink),
)


def run_checks(checks=CHECKS) -> list:
    results = []
    for name, check in checks:
        try:
            status, detail = check()
        except Exception as exc:  # a broken check must report, not abort the others
            status, detail = FAIL, f"{type(exc).__name__}: {exc}"
        results.append((name, status, detail))
    return results


def main() -> None:
    results = run_checks()
    width = max(len(name) for name, _, _ in results)
    for name, status, detail in results:
        print(f"  {name:<{width}}  {status:4}  {detail}")
    failed = [name for name, status, _ in results if status == FAIL]
    print(f"\n{len(results) - len(failed)}/{len(results)} checks passed" +
          (f"; FAILED: {', '.join(failed)}" if failed else ""))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
