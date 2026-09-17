"""`ros2 run aero_sense_bringup spawn -world W ...` — ros_gz_sim create, once the world is up.

create asks Gazebo once and gives up after about five seconds. The disaster world takes longer
than that to load, so a spawn launched alongside it times out, and whether Gazebo later acts on
the stale request is luck: the drone came up in one launch and not the next, leaving SITL waiting
for a vehicle that never existed. So wait for the world's clock (it ticks only once loading is
done), then create, and ask again if the request still timed out. A repeat of a request Gazebo
did act on is refused as a duplicate name, which is harmless.
"""
import subprocess
import sys

ATTEMPTS = 5
CLOCK_WAIT_S = 30


def timed_out(output: str) -> bool:
    return "timed out" in output


def main() -> int:
    args = sys.argv[1:]
    world = args[args.index("-world") + 1]
    clock = ["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/clock"]
    while True:
        try:
            if subprocess.run(clock, capture_output=True, timeout=CLOCK_WAIT_S).returncode == 0:
                break
        except subprocess.TimeoutExpired:
            print(f"spawn: still waiting for world {world} to load", flush=True)
    for _ in range(ATTEMPTS):
        result = subprocess.run(["ros2", "run", "ros_gz_sim", "create", *args], capture_output=True, text=True)
        output = result.stdout + result.stderr
        if not timed_out(output):
            print(output, end="", flush=True)
            return result.returncode
        print("spawn: create timed out; asking again", flush=True)
    print(output, end="", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
