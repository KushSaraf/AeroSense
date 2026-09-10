# Triage engine output for one victim. Decision support only.

float32 score                    # 0..1, higher = more urgent
string priority                  # P1 | P2 | P3

# Inputs, each normalised to 0..1 unless stated
float32 survivor_probability     # fused detection confidence
float32 thermal_consistency      # how stable the body-heat signature is across frames
float32 movement_score           # 0 = no movement seen, 1 = walking/waving
float32 hazard_severity          # severity of the worst nearby hazard
float32 danger                   # proximity factor of that hazard
float32 time_to_hazard_s         # predicted time until the hazard reaches the victim (inf = never)
float32 accessibility            # 1 = straight safe ground route, 0 = unreachable
float32 detection_confidence

string rationale                 # human-readable reason, e.g. "strong thermal, active fire 3 m, no egress"
