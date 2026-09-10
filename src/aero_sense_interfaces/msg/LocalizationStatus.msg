# Which source is localising the drone, and how well.

std_msgs/Header header
string source                    # GPS | VIO | DEAD_RECKONING
string gps_status                # OK | DEGRADED | LOST
float32 gps_hacc_m
uint8 gps_satellites
string vio_status                # STANDBY | ACTIVE | LOST
float32 vio_quality              # 0..1 (tracked-feature ratio)
float32 position_uncertainty_m
