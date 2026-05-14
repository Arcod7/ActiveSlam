Visualise point cloud with rviz:

```bash
# 1. Run the point cloud publisher
ros2 launch basic_slam step2_pointcloud.launch.py

# 2. In another terminal, run rviz
rviz2
# Put Camera and /cloud_in with 1 as size in meter, sphere and BEST_EFFORT for Reliability

# 3. Run the keyboard teleop to move the robot around
ros2 run launch_tools my_keyboard
```

Octomap:

```bash
# 1. Run the octomap server, step1(odom to tf) and step2 (depth to point cloud)
ros2 launch basic_slam step3_octomap.launch.py

# 2. In another terminal, run rviz
rviz2
# Put Map and /octomap with 1 as size in meter, cube and BEST_EFFORT for Reliability

# 3. Run the keyboard teleop to move the robot around
ros2 run launch_tools my_keyboard
```
