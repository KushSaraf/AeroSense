"""How much gas is in the air at a point: a Gaussian plume, for the simulator's gas sensors.

Simulator side, like `rangefinder_sim`'s ray casting: the drone never runs this, it finds gas with
its sensors. Pure maths, no ROS.

The steady-state Gaussian plume with reflection off the ground,

    C = Q / (2 pi u sy sz) * exp(-y^2 / 2 sy^2) * [exp(-(z - H)^2 / 2 sz^2) + exp(-(z + H)^2 / 2 sz^2)]

with sy, sz from Briggs' open-country fits (Briggs 1973) for the Pasquill-Gifford stability
class. What it leaves out, and so what the simulation cannot show: buoyancy (ammonia rises, H2S
sinks), buildings in the way, gusts - it is the time-averaged plume, not the puffs a sensor
really sees - and anything upwind, which reads zero. Briggs' fits are for 100 m to 10 km; nearer
the source they are an extrapolation.
"""
import math

#: Briggs (1973) open-country dispersion, class -> (sigma_y(x), sigma_z(x)) in metres.
BRIGGS_RURAL = {
    "A": (lambda x: 0.22 * x / math.sqrt(1 + 0.0001 * x), lambda x: 0.20 * x),
    "B": (lambda x: 0.16 * x / math.sqrt(1 + 0.0001 * x), lambda x: 0.12 * x),
    "C": (lambda x: 0.11 * x / math.sqrt(1 + 0.0001 * x), lambda x: 0.08 * x / math.sqrt(1 + 0.0002 * x)),
    "D": (lambda x: 0.08 * x / math.sqrt(1 + 0.0001 * x), lambda x: 0.06 * x / math.sqrt(1 + 0.0015 * x)),
    "E": (lambda x: 0.06 * x / math.sqrt(1 + 0.0001 * x), lambda x: 0.03 * x / (1 + 0.0003 * x)),
    "F": (lambda x: 0.04 * x / math.sqrt(1 + 0.0001 * x), lambda x: 0.016 * x / (1 + 0.0003 * x)),
}
#: g/mol, for mg/m^3 -> ppm.
MOLAR_MASS = {"CO": 28.01, "NO2": 46.01, "NH3": 17.03, "H2S": 34.08}
#: Litres a mole of gas takes at 25 C and 1 atm: ppm = mg/m^3 * this / molar mass.
MOLAR_VOLUME_L = 24.45
#: Closer to the source than this the plume formula divides by nothing; call it the source's size.
MIN_DOWNWIND_M = 1.0
#: Calm air does not carry a plume anywhere, and the formula blows up at u = 0.
MIN_WIND_MPS = 0.5


def downwind(from_deg: float) -> tuple:
    """Unit vector (east, north) the wind blows towards, from a meteorological FROM bearing."""
    to = math.radians(from_deg + 180.0)
    return math.sin(to), math.cos(to)


def concentration_ppm(source: dict, wind: dict, x: float, y: float, z: float) -> float:
    """ppm of `source["species"]` at map point (x, y, z), z above the ground the plume reflects off."""
    east, north = downwind(wind["from_deg"])
    dx, dy = x - source["x"], y - source["y"]
    along = dx * east + dy * north
    if along < MIN_DOWNWIND_M:
        return 0.0
    across = -dx * north + dy * east
    sigma_y_of, sigma_z_of = BRIGGS_RURAL[wind["stability"]]
    sy, sz = sigma_y_of(along), sigma_z_of(along)
    height = source["z"]
    grams_per_m3 = (source["rate_g_per_s"] / (2 * math.pi * max(wind["speed_mps"], MIN_WIND_MPS) * sy * sz)
                    * math.exp(-across ** 2 / (2 * sy ** 2))
                    * (math.exp(-(z - height) ** 2 / (2 * sz ** 2)) + math.exp(-(z + height) ** 2 / (2 * sz ** 2))))
    return grams_per_m3 * 1000.0 * MOLAR_VOLUME_L / MOLAR_MASS[source["species"]]


def reading(ppm: float, range_ppm) -> tuple:
    """(what the sensor reports, saturated): nothing below its floor, pinned at the top of its range."""
    floor, ceiling = range_ppm
    if ppm < floor:
        return 0.0, False
    if ppm >= ceiling:
        return float(ceiling), True
    return float(ppm), False
