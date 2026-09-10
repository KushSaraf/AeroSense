# One tracked, geo-located, triaged victim.

std_msgs/Header header           # frame_id "map"; stamp = latest observation
string victim_id                 # persistent tracker id, e.g. "V007"

geometry_msgs/Point position     # map frame (ENU, metres)
float64 latitude
float64 longitude
float64 altitude

float32 confidence               # fused P(survivor)
float32 rgb_confidence
float32 thermal_confidence
string evidence                  # MULTI-SENSOR CONFIRMED | THERMAL ONLY | RGB ONLY

float32 thermal_strength         # 0..1
string movement                  # static | walking | waving | unknown
string pose_state                # lying | sitting | standing | unknown
float32 hazard_proximity         # 0..1
float32 accessibility            # 0..1
uint32 observations              # frames this track was seen in

string priority                  # P1 | P2 | P3 (copy of triage.priority for filtering)
TriageScore triage
