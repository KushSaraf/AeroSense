# Complete current list of ground routes, one per casualty (not a delta).
std_msgs/Header header
SafeRoute[] routes
TeamRoute[] teams                # which team goes to whom (team_plan.py)
string[] unassigned              # casualties no team reaches within its shift, or none can reach
