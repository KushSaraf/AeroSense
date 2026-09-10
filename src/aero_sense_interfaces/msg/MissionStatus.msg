# Mission state machine status. state is one of:
#   STANDBY PRE_FLIGHT TAKEOFF TRANSIT SEARCHING VICTIM_DETECTED HAZARD_DETECTED
#   LOCAL_REPLAN GPS_DENIED OFFLINE_AUTONOMY RETURNING LANDING MISSION_COMPLETE EMERGENCY

std_msgs/Header header
string mission_id
string scenario                  # full | earthquake | flood | fire | landslide | chemical
string state
string previous_state
string reason                    # event that caused the last transition
string sector                    # sector the drone is currently over

float32 coverage_percent
uint32 victims_detected
uint32 p1_count
uint32 p2_count
uint32 p3_count
uint32 hazards_detected
float32 elapsed_s
