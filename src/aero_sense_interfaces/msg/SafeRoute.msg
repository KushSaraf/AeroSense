# Recommended ground-responder route from an entry point to a victim.

std_msgs/Header header
string victim_id
geometry_msgs/Point entry_point  # map frame
nav_msgs/Path path               # map frame waypoints
float32 distance_m
float32 estimated_time_s
string risk                      # LOW | MODERATE | HIGH (worst cost class crossed)
float32 cost
bool reachable                   # false if every route crosses a CRITICAL hazard
