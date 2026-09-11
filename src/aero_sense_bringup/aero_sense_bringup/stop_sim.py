"""`ros2 run aero_sense_bringup stop_sim` — stop every part of a running simulation.

ArduPilot SITL ignores SIGTERM and Gazebo can outlive Ctrl-C, and a half-dead simulation breaks
the next launch silently: SITL cannot bind its port, so the world comes up with a drone that no
autopilot flies. This stops the lot and says whether the port came free.

Processes are matched on the program being run, never on a command line that merely mentions
one: a shell whose arguments contain "gz sim" is not a simulation, and killing it would take the
caller down with it.
"""
import os
import signal
import socket
import sys
import time
from pathlib import Path

#: Programs that are part of a simulation, by the name they are invoked as.
SIMULATION_PROGRAMS = {"arducopter", "mavproxy.py", "parameter_bridge",
                       "static_transform_publisher", "rviz2"}
#: Python processes count only when they are running one of our nodes or the launch itself.
PYTHON_MARKERS = ("aero_sense", "mavproxy.py")
SITL_TCP_PORT = 5760
GRACE_S = 2.0


def _is_simulation(argv: list) -> bool:
    if not argv:
        return False
    program = Path(argv[0]).name
    if program == "gz":
        return "sim" in argv                      # `gz sim`, not `gz topic`
    if program in SIMULATION_PROGRAMS:
        return True
    if program.startswith("python"):
        rest = " ".join(argv[1:])
        return any(marker in rest for marker in PYTHON_MARKERS)
    return False


def simulation_pids(processes, exclude=()) -> list:
    """`processes`: (pid, argv) pairs. Excluded pids (us, and our ancestors) are never returned."""
    excluded = set(exclude)
    return [pid for pid, argv in processes if pid not in excluded and _is_simulation(argv)]


def running_processes():
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = (entry / "cmdline").read_bytes().decode().split("\0")
        except OSError:                            # it exited while we looked
            continue
        yield int(entry.name), [part for part in argv if part]


def own_process_tree() -> set:
    """Our pid and every ancestor, so stopping a simulation never stops the caller."""
    pids, pid = set(), os.getpid()
    while pid > 1:
        pids.add(pid)
        try:
            status = Path(f"/proc/{pid}/status").read_text()
        except OSError:
            break
        parent = [line for line in status.splitlines() if line.startswith("PPid:")]
        pid = int(parent[0].split()[1]) if parent else 0
    return pids


def port_is_free(port: int = SITL_TCP_PORT) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError:
            return False
    return True


def main() -> None:
    ours = own_process_tree()
    stopped = []
    for sig in (signal.SIGTERM, signal.SIGKILL):
        pids = simulation_pids(running_processes(), exclude=ours)
        if not pids:
            break
        for pid in pids:
            try:
                os.kill(pid, sig)
                stopped.append(pid)
            except ProcessLookupError:
                pass
        time.sleep(GRACE_S)

    left = simulation_pids(running_processes(), exclude=ours)
    free = port_is_free()
    if left or not free:
        print(f"still running: {len(left)} process(es); port {SITL_TCP_PORT} "
              f"{'free' if free else 'busy'}", file=sys.stderr)
        sys.exit(1)
    print(f"simulation stopped ({len(set(stopped))} processes); port {SITL_TCP_PORT} is free")


if __name__ == "__main__":
    main()
