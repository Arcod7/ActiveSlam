"""Values more than one node has to agree on, defined once.

A shared number written separately in each node drifts silently: the planner
validated arrival at 2 m while the motion executor parked at 4 m, so no goal
was ever reached by arriving — only the 30 s stuck timer ever released one.
Anything here is imported, never copied, including by frontier_slam.launch.py
for its argument defaults.
"""

# How close counts as reaching a goal, in metres. The motion executor stops
# driving here and the planner validates arrival at the same radius; a planner
# radius smaller than the executor's can never be satisfied. Exposed for tuning
# as the goal_radius_m launch argument.
GOAL_RADIUS_M = 4.0
