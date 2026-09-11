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
                         cwd=str(WORKSPACE))
    return {"started": True, "log": str(log_path), **status()}


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
