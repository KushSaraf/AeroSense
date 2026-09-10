# One mapped hazard region.

std_msgs/Header header           # frame_id "map"; stamp = latest observation
string hazard_id                 # persistent id, e.g. "H012"
string type                      # fire | smoke | flood | landslide | chemical | debris | structural | electrical
string severity                  # CRITICAL | HIGH | MODERATE | SAFE
float32 confidence
string sector                    # earthquake | flood | fire | landslide | chemical

HazardPolygon footprint
geometry_msgs/Point centroid     # map frame
float64 latitude                 # centroid in WGS84
float64 longitude
float32 area_m2
