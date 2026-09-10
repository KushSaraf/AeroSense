# Hazard footprint as a closed ring (last vertex connects back to the first).
geometry_msgs/Polygon polygon    # map frame (ENU, metres)
float64[] latitudes              # same ring in WGS84, index-aligned with polygon.points
float64[] longitudes
