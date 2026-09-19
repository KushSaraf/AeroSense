# One ground team's tour: from the command base to its casualties in order, and back.
string team                      # T1, T2, ...
string[] victim_ids              # in the order the team reaches them
nav_msgs/Path path               # map frame, along the roads
float32 distance_m
float32 estimated_time_s         # travel, time on site with each casualty, and back to base
