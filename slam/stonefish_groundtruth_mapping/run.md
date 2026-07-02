Visualise point cloud with rviz:

```bash
# 1. Run the point cloud publisher
ros2 launch stonefish_groundtruth_mapping pointcloud.launch.py

# 2. In another terminal, run rviz
rviz2
# Put Camera and /cloud_in with 1 as size in meter, sphere and BEST_EFFORT for Reliability

# 3. Run the keyboard teleop to move the robot around
ros2 run launch_tools my_keyboard
```

Octomap:

```bash
# 1. Run the full stack: Stonefish + TF + point cloud + octomap_server
ros2 launch stonefish_groundtruth_mapping octomap.launch.py

# 2. In another terminal, run rviz
rviz2
# Put Map and /octomap with 1 as size in meter, cube and BEST_EFFORT for Reliability

# 3. Run the keyboard teleop to move the robot around
ros2 run launch_tools my_keyboard
```
