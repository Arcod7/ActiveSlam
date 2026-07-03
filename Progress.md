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

**Observed impact**: 🔲 Not yet fully tested with sensor models. Loader function tested via Python script successfully.
