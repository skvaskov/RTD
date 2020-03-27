# Experiment **1**

This experiment determines the buffer size to be used for RRT and NMPC planners for the Segway and Rover. By "buffer size" we mean how much to dilate, or pad, obstacles, to prevent collisions.

To let the buffer size be the only variable affecting performance, we give the robots perfect sensor information of their entire world, and a large amount of time to plan per receding-horizon iteration.

Each planner is run on 1000 worlds, with varying buffer sizes (see, e.g., `experiment_1_segway.m` ). Then the resulting total number of crashes and goals reached is tallied (see `experiment_1_segway_inspect_results.m`).

