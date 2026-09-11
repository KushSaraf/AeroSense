"""World helpers shared by the launch files and system_check: where a drone spawns in a world,
and where the world's models are found."""
import math
import os
import socket
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

SPAWN_FRAME = "drone_spawn"
#: base_link height of the iris standing on its landing gear.
GEAR_HEIGHT_M = 0.195
#: GitHub asset repos used in place (gitignored, not vendored). See README "Reference assets".
REFERENCE_DIR = Path(os.environ.get("AERO_SENSE_REFERENCE", Path.home() / "sih_2026" / "reference"))
REFERENCE_MODEL_DIRS = ("tdf_gazebo-main/models", "gazebo_models_worlds_collection-master/models",
                        "Autonomous-robot-for-fire-detection-main/models",
                        "darpa_subt_worlds-main/worlds/models")

def port_is_free(port: int, host: str = "127.0.0.1") -> bool:
    """Whether a TCP port can still be bound.

    SITL exits immediately if its port is taken, and a launch that ignores this leaves a world
    with a drone in it that no autopilot is flying — which looks like "the drone is broken"
    rather than "a simulation is already running".
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind((host, port))
        except OSError:
            return False
    return True


def world_name(world: Path) -> str:
    return ET.parse(world).getroot().find("world").get("name")


def spawn_pose(world: Path) -> tuple:
    """(x, y, z, yaw) of a drone standing on the world's `drone_spawn` frame (ENU, yaw rad)."""
    pose = ET.parse(world).getroot().find(f"world/frame[@name='{SPAWN_FRAME}']/pose")
    if pose is None:
        raise ValueError(f"{world} has no <frame name='{SPAWN_FRAME}'><pose>")
    x, y, z, _, _, yaw = (float(v) for v in pose.text.split())
    if pose.get("degrees") == "true":
        yaw = math.radians(yaw)
    return x, y, z + GEAR_HEIGHT_M, yaw


def origin(world: Path) -> tuple:
    """(lat, lon, elevation) of the world origin from <spherical_coordinates>. SITL gets this as
    its home, so ArduPilot's local NED origin is the world origin and the map frame is the
    Gazebo world frame (JSON positions are relative to the SITL home)."""
    sc = ET.parse(world).getroot().find("world/spherical_coordinates")
    if sc is None:
        raise ValueError(f"{world} has no <spherical_coordinates>")
    return tuple(float(sc.find(tag).text) for tag in ("latitude_deg", "longitude_deg", "elevation"))


def reference_model_paths() -> list:
    paths = [REFERENCE_DIR / d for d in REFERENCE_MODEL_DIRS]
    missing = [str(p) for p in paths if not p.is_dir()]
    if missing:
        raise FileNotFoundError(f"reference assets missing: {', '.join(missing)} "
                                "(clone them or set AERO_SENSE_REFERENCE; see README)")
    return paths


def reference_texture_dirs(model_paths: list) -> list:
    """Each reference model's materials/textures. Their meshes name textures by bare filename
    (Gazebo Classic found them through OGRE resource groups); Harmonic finds bare names only
    on the resource path.
    ponytail: first match wins, so the tdf models sharing a name with different pixels
    (rubble_diffuse.jpg, rubble_spec.jpg, bark_diffuse.png) all get one variant. Copy the
    textures into per-model wrappers if that ever shows."""
    return sorted(d for p in model_paths for d in p.glob("*/materials/textures") if d.is_dir())


def resource_paths() -> list:
    """GZ_SIM_RESOURCE_PATH entries: our models, the ArduPilot iris, the reference assets."""
    ap_gazebo = Path(get_package_share_directory("ardupilot_gazebo"))
    reference = reference_model_paths()
    return [
        Path(get_package_share_directory("aero_sense_description")) / "models",
        Path(get_package_share_directory("aero_sense_gazebo")) / "models",
        ap_gazebo / "models",
        ap_gazebo.parent,                   # resolves package://ardupilot_gazebo/... meshes
        *reference,
        *reference_texture_dirs(reference),
        REFERENCE_DIR,                      # our models name loose textures model://<repo>/<path>
    ]


def included_models(world: Path) -> set:
    """Names of the `model://<name>` includes in a world (Fuel URLs excluded)."""
    uris = (u.text.strip() for u in ET.parse(world).getroot().iter("uri"))
    return {u[len("model://"):].split("/")[0] for u in uris if u.startswith("model://")}
