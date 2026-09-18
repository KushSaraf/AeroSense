# Vehicle telemetry as reported by the autopilot over MAVLink.

std_msgs/Header header
string drone_id                  # e.g. "AS-01"
string mode                      # autopilot flight mode
bool armed

float32 battery_percent
float32 battery_voltage
float32 altitude_m               # above home
float32 ground_speed_mps
float64 latitude
float64 longitude
geometry_msgs/Pose pose          # map frame

string gps_status                # OK | DEGRADED | LOST
float32 gps_hacc_m
string vio_status                # STANDBY | ACTIVE | LOST
string navigation                # GPS | VISION | NONE: what the EKF takes its position from
string link_status               # CONNECTED | DEGRADED | OFFLINE | RECOVERING | SYNCHRONIZING
