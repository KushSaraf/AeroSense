# Ground-link state and store-and-forward queue depth.

std_msgs/Header header
string state                     # CONNECTED | DEGRADED | OFFLINE | RECOVERING | SYNCHRONIZING
float32 link_quality             # 0..1
bool offline_autonomy            # true while the drone operates without the ground

uint32 queued_p1
uint32 queued_p2
uint32 queued_p3
uint32 queued_map
uint32 queued_media
float32 offline_duration_s
