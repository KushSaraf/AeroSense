"""Starting and stopping the simulation from the dashboard.

The bridge outlives any one simulation: it runs on its own, reports whether a simulation is up,
and can start or stop one on request. That is what lets an operator open the dashboard, create a
mission and press start, instead of running launch files by hand first.

Local control only. The bridge binds 127.0.0.1 by default because these endpoints start
processes on the machine that serves them.
"""
import os
import subprocess
import time
from pathlib import Path

from aero_sense_bringup import stop_sim

#: Where the workspace lives, so the launch can be sourced the same way a terminal would.
WORKSPACE = Path(os.environ.get("AERO_SENSE_WORKSPACE", Path.home() / "sih_2026"))
LOG_DIR = WORKSPACE / "logs" / "dashboard"
ROS_SETUP = "/opt/ros/humble/setup.bash"
UAV_SETUP = str(Path.home() / "uav_ws" / "install" / "setup.bash")
START_TIMEOUT_S = 240
#: How long a freshly opened window must survive before it counts as open. Qt aborts within a
#: second when it cannot initialise, so this catches that without making the button feel slow.
VIEWER_SETTLE_S = 4.0
#: OpenCV's pip wheel points Qt at its own bundled plugins the moment it is imported, and the
#: bridge imports it for the camera stream. Every window the bridge starts inherited that and
#: aborted in Qt's platform init, so Gazebo and RViz never opened from the dashboard.
POLLUTING_QT_VARS = ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR")


def _child_env(extra: dict | None = None) -> dict:
    """The environment a spawned process should see: ours, minus what OpenCV injected."""
    env = {key: value for key, value in os.environ.items() if key not in POLLUTING_QT_VARS}
    env.update(extra or {})
    return env


def _gazebo_resource_env() -> dict:
    """Model paths for a Gazebo window. The GUI loads meshes itself, so without the same paths
    the server has it opens onto a world of missing models."""
    try:
        from aero_sense_bringup import worlds
        paths = [str(path) for path in worlds.resource_paths()]
    except Exception:                        # reference assets missing: the GUI still opens
        return {}
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    return {"GZ_SIM_RESOURCE_PATH": ":".join(paths + ([existing] if existing else []))}


def _log_tail(path: Path, lines: int = 3) -> str:
    try:
        return " | ".join(path.read_text(errors="ignore").strip().splitlines()[-lines:])
    except OSError:
        return ""


def _launch_command(quality: str, gui: bool, rviz: bool, cruise_speed: float) -> str:
    launch = (f"ros2 launch aero_sense_bringup full_system.launch.py quality:={quality} "
              f"gui:={'true' if gui else 'false'} rviz:={'true' if rviz else 'false'} "
              f"cruise_speed:={cruise_speed} dashboard:=false")
    return (f"source {ROS_SETUP} && source {UAV_SETUP} && "
            f"source {WORKSPACE}/install/setup.bash && {launch}")


def simulation_processes() -> list:
    """Pids belonging to a running simulation (never this process or its parents)."""
    return stop_sim.simulation_pids(stop_sim.running_processes(), exclude=stop_sim.own_process_tree())


def status() -> dict:
    running = simulation_processes()
    return {
        "running": bool(running),
        "processes": len(running),
        "port5760Free": stop_sim.port_is_free(),
    }


def start(quality: str = "low", gui: bool = True, rviz: bool = False,
          cruise_speed: float = 8.0) -> dict:
    """Start a simulation, unless one is already up.

    Refusing rather than starting a second one is deliberate: two simulations publish the same
    topics and quietly corrupt everything the dashboard shows.
    """
    if simulation_processes():
        return {"started": False, "reason": "a simulation is already running", **status()}
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"simulation-{time.strftime('%Y%m%d-%H%M%S')}.log"
    with log_path.open("w") as log:
        subprocess.Popen(["bash", "-lc", _launch_command(quality, gui, rviz, cruise_speed)],
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                         cwd=str(WORKSPACE), env=_child_env())
    return {"started": True, "log": str(log_path), **status()}


def restart(quality: str = "low", gui: bool = True, rviz: bool = False,
            cruise_speed: float = 8.0) -> dict:
    """Stop whatever is running and bring a fresh simulation up.

    The way out of a mission that has gone wrong — a crashed drone, a wedged autopilot —
    without leaving the dashboard for a terminal.
    """
    stopped = stop()
    if not stopped["port5760Free"]:
        return {"started": False, "reason": "the old simulation did not release port 5760",
                **status()}
    return start(quality=quality, gui=gui, rviz=rviz, cruise_speed=cruise_speed)


#: Windows an operator can open onto a running simulation, and how to tell one is already up.
VIEWERS = {
    "gazebo": {"command": "gz sim -g -v2", "match": "gz sim -g"},
    "rviz": {"command": "rviz2 -d $(ros2 pkg prefix aero_sense_visualization)"
                        "/share/aero_sense_visualization/config/aero_sense.rviz",
             "match": "rviz2"},
}


def viewer_running(kind: str) -> bool:
    match = VIEWERS[kind]["match"]
    for _pid, argv in stop_sim.running_processes():
        line = " ".join(argv)
        if line.startswith(match) or f"/{match}" in line.split(" ")[0]:
            return True
    return False


def open_viewer(kind: str) -> dict:
    """Open Gazebo or RViz onto the running simulation.

    Both attach to what is already running rather than starting their own: a second Gazebo
    server would publish the same topics and corrupt the mission underway.
    """
    if kind not in VIEWERS:
        return {"opened": False, "reason": f"unknown view {kind!r}"}
    if not simulation_processes():
        return {"opened": False, "reason": "no simulation is running"}
    if viewer_running(kind):
        return {"opened": False, "reason": f"{kind} is already open"}
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"{kind}-{time.strftime('%Y%m%d-%H%M%S')}.log"
    command = (f"source {ROS_SETUP} && source {UAV_SETUP} && "
               f"source {WORKSPACE}/install/setup.bash && {VIEWERS[kind]['command']}")
    env = _child_env(_gazebo_resource_env() if kind == "gazebo" else None)
    with log_path.open("w") as log:
        process = subprocess.Popen(["bash", "-lc", command], stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, cwd=str(WORKSPACE), env=env)
    # a window that dies on startup must not be reported as open
    try:
        code = process.wait(timeout=VIEWER_SETTLE_S)
    except subprocess.TimeoutExpired:
        return {"opened": True, "view": kind, "log": str(log_path)}
    return {"opened": False, "view": kind, "log": str(log_path),
            "reason": f"{kind} exited on startup (code {code}): {_log_tail(log_path)}"}


def stop() -> dict:
    """Stop everything, including the parts that outlive a terminal's Ctrl-C."""
    ours = stop_sim.own_process_tree()
    for signal_number in (15, 9):
        pids = stop_sim.simulation_pids(stop_sim.running_processes(), exclude=ours)
        if not pids:
            break
        for pid in pids:
            try:
                os.kill(pid, signal_number)
            except ProcessLookupError:
                pass
        time.sleep(2.0)
    stop_sim.stop_ros_daemon()
    return {"stopped": True, **status()}
