What's missing for real SLAM
A real SLAM system replaces the perfect odometry with a pose estimated from sensor data:

1. Odometry estimation — compare consecutive point clouds (ICP scan matching) or use DVL/IMU to get a pose estimate. It drifts over time.
2. Loop closure detection — recognize when the robot revisits a known place, and use that to correct accumulated drift.
3. Pose graph optimization — treat all pose estimates and loop closures as a graph, then optimize the whole trajectory.
4. Map update — rebuild the map with the corrected poses.
Practical next step
Rather than implementing all that from scratch, the typical approach is to plug in an existing SLAM package and compare its output against your ground-truth OctoMap:

- rtabmap_ros — most practical for a depth camera underwater, does full 6-DOF SLAM with loop closure
- slam_toolbox — simpler, 2D, good for flat environments
- hdl_graph_slam — point-cloud-based, good for 3D structure like your offshore station
