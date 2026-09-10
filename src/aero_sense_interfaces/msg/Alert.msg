# Real-time alert. type is one of:
#   SURVIVOR_DETECTED P1_SURVIVOR P2_SURVIVOR P3_SURVIVOR ACTIVE_FIRE FLOOD_DETECTED
#   LANDSLIDE_DETECTED CHEMICAL_LEAK UNSTABLE_STRUCTURE GPS_LOST VIO_ACTIVE
#   COMMUNICATION_LOST LOW_BATTERY OBSTACLE_DETECTED ROUTE_REPLANNED

string id
string type
string severity                  # CRITICAL | HIGH | MODERATE | INFO
string message
string rationale
float64 latitude
float64 longitude
geometry_msgs/Point position     # map frame
float32 confidence
string related_entity            # victim or hazard id, empty if none
builtin_interfaces/Time timestamp
