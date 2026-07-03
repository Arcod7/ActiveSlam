# ActiveSlam SLAM Backend — Change Log

Each entry records a change, its objective, and the **observed impact** once tested.
Entries are ordered chronologically. Mark impact as ✅ positive, ⚠️ mixed/partial,
❌ negative (reverted), or 🔲 not yet tested.

Current parameters → `STATE.md`

---

## Phase 1 — Package scaffolding and dependencies

**Date**: 2026-07-03  
**Files**: `requirements.txt`, `slam_backend/`, `eval_tools/`

**Objective**: Set up the initial ROS 2 Python packages and update `requirements.txt` with `gtsam` and `small-gicp`.

**Observed impact**: ✅ Python packages installed and `colcon build` succeeded.

---

## Phase 2 — Noise Profiles

**Date**: 2026-07-03  
**Files**: `slam_backend/sensor_models/noise_profiles.py`, `slam_backend/config/noise_*.yaml`

**Objective**: Implemented the `NoiseProfile` dataclasses and the three YAML configurations (ideal, realistic, degraded).

**Observed impact**: ✅ Not yet fully tested with sensor models. Loader function tested via Python script successfully.

---

## Phase 3 & 4 — Sensor Simulators & sensors_only.launch.py

**Date**: 2026-07-03  
**Files**: `pressure_sim.py`, `imu_sim.py`, `dvl_sim.py`, `sensors_only.launch.py`

**Objective**: Implemented the standalone ROS 2 sensor simulator nodes. They subscribe to `/StoneFish/Odometry` and publish to `/slam/sensors/pressure_depth`, `/slam/sensors/imu_orientation`, and `/slam/sensors/dvl_odom` with their respective noise profiles applied. Built `sensors_only.launch.py` to launch all three with a configurable `noise_profile`.

**Observed impact**: 🔲 Compiled. Waiting for execution test.
