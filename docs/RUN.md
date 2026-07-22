# Run

Full reference for the launcher and `demo.launch.py`. For a guided tour that
builds up one capability at a time, see [`DEMOS.md`](DEMOS.md); for the short
version, the root [`README.md`](../README.md).

Everything below assumes the workspace is built and sourced:

```bash
source <workspace>/.venv/bin/activate
source <workspace>/install/setup.zsh
```

## The launcher

```bash
python3 launcher.py
```

Opens a workspace menu (**Launch · Update · Rebuild · Infos · Exit**), where
*Infos* summarises the technologies the project is built on. *Launch* opens a
control screen that:

- **changes options while the stack runs.** The stack is split into
  independently restartable layers, so switching the mapper, the mode or the
  pose source only bounces the layers that depend on that option — Stonefish,
  the slow part, keeps running. The screen shows which groups a pending change
  will restart before you apply it.
- **explains each option** in a description pane as you move through them.
- **arms motion** (press `m`) and shows the safety gate's live state in the
  header. The gate is fail-closed and starts disabled, so nothing moves until
  it is armed — with RViz off, this is the only way to arm it.
- **resets the run** (press `r`): the vehicle is teleported back to its
  scenario spawn pose via Stonefish's `respawn_robot` service, and the map,
  SLAM pose graph, eval output and planner blacklist are all cleared. The
  simulator keeps running throughout.
- **includes keyboard teleop** (press `t`). It publishes to
  `/motion/body_command` with the same keys as `ros2 run launch_tools
  my_keyboard`, so `mode:=teleop` does not need a second terminal. In frontier
  mode it suspends the planner first — the safety gate treats two publishers on
  that topic as `MULTIPLE_COMMAND_SOURCES` and stops all motion.
- **retunes wall-following parameters live**, via `ros2 param set`, with no
  restart at all — those parameters are re-read every control cycle. Options
  that support this are marked `(live)`.
- **shuts down cleanly.** Every layer runs in its own process group and is torn
  down with an escalating SIGINT → SIGTERM → SIGKILL, on quit, on Ctrl-C, and
  on exit, so nothing is left running in the background.

Per-layer logs are written to `logs/launcher/` (gitignored). Selections persist
in `~/.activeslam_launcher.json`, so it reopens on the last configuration used.

The launcher drives the same launch files as `demo.launch.py` below.

## `demo.launch.py`

One command brings up Stonefish + TF + point cloud + mapper + RViz:

```bash
ros2 launch bringup demo.launch.py
```

Independent switches, each defaulting to the first value:

```bash
ros2 launch bringup demo.launch.py mode:=teleop|frontier          # operator mode
ros2 launch bringup demo.launch.py mapper:=octomap|tsdf           # map backend
ros2 launch bringup demo.launch.py slam:=none|slam                # pose source
ros2 launch bringup demo.launch.py rviz:=false                    # headless
ros2 launch bringup demo.launch.py noise_profile:=realistic|ideal|sonar_only|odom_pos_only|odom_only|degraded  # slam:=slam only
ros2 launch bringup demo.launch.py mode:=frontier scan_style:=sweep|spin \
  scan_sweep_deg:=180.0                                           # cable-safe sweep (default) vs full revolution
ros2 launch bringup demo.launch.py mode:=frontier safety_start_enabled:=true  # arm the motion gate at startup
ros2 launch bringup demo.launch.py mode:=frontier motion:=walloriented \
  wall_orientation_offset_deg:=30                                 # follow path, look toward nearest wall
```

`mapper:=tsdf` needs vdbfusion — `./bootstrap.sh --with-vdbfusion`.

Wall-guided path executors (`motion:=walloriented|walllooking`) are outside the
evaluated scope of this project — see [`FUTURE_WORK.md`](FUTURE_WORK.md).

### Motion safety

The motion safety gate is **fail-closed**: nothing moves until `/motion/enable`
is published, whether that comes from the RViz panel, the launcher's `m` key,
or `safety_start_enabled:=true`. `/motion/safety_status` says what is blocking.

### Teleop needs its own terminal

`mode:=teleop` (the default) brings up sim+mapper and prints a reminder to run
teleop yourself in another terminal:

```bash
ros2 run launch_tools my_keyboard
```

Raw keyboard input needs a real terminal — `ros2 launch` cannot hand one to a
launched node, so this cannot be bundled into the single command above.
`mode:=frontier` does not have this problem and launches fully inside
`demo.launch.py`. The launcher's `t` key avoids the second terminal entirely.

## Benchmarking

`slam:=none` (default) broadcasts pose from ground truth. `slam:=slam` replaces
that with a GTSAM iSAM2 pose graph correcting simulated pressure/IMU/DVL/sonar
sensor noise (see [`slam/slam_backend`](../slam/slam_backend)), and starts a
benchmark node that logs ATE/RPE against ground truth plus a map_metrics node
that scores the belief map against the ground-truth reference map (IoU/coverage
for `mapper:=octomap`, chamfer distance/coverage for `mapper:=tsdf`), writing
TUM trajectory files + CSVs to `eval/runs/<timestamp>/`.

Switches, all `slam:=slam` only, all defaulting to current behaviour:

```bash
ros2 launch bringup demo.launch.py slam:=slam loop_closure:=false             # A/B: no loop closure
ros2 launch bringup demo.launch.py slam:=slam mapper:=tsdf map_rebuild:=true  # rebuild belief TSDF after big closures
ros2 launch bringup demo.launch.py slam:=slam noise_seed:=7                   # reproducible, decorrelated noise draws
ros2 launch bringup demo.launch.py slam:=slam output_dir:=/path/to/run        # label eval output instead of a timestamp
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier revisit:=true    # break off exploration to close loops when uncertainty grows
```

Plot a finished run:

```bash
ros2 run eval_tools plot_results eval/runs/<timestamp>/
```

To compare configurations/seeds unattended, use the batch orchestrator (a plain
script, run with `python3` after sourcing the workspace — see
[`eval/eval_tools/scripts/run_matrix.py`](../eval/eval_tools/scripts/run_matrix.py)
and the example matrices in [`eval/eval_tools/config/`](../eval/eval_tools/config)):

```bash
python3 eval/eval_tools/scripts/run_matrix.py eval/eval_tools/config/matrix_smoke.yaml  # ~5min pre-flight check
python3 eval/eval_tools/scripts/run_matrix.py eval/eval_tools/config/matrix_full.yaml   # 35 runs, ~5.3h
```

## RViz views

The view switches automatically with `mapper`/`slam` (`slam:=slam` wins if both
apply — see [`bringup/rviz/`](../bringup/rviz)):

- default (`mapper:=octomap slam:=none`): the base view.
- `mapper:=tsdf`: TSDF surface/voxels in place of the OctoMap displays (which
  would just sit empty — `octomap_server` is not launched in this mode).
- `slam:=slam`: ground truth (green) vs SLAM (blue) vs raw dead-reckoning (red)
  paths, a live drift arrow + text HUD (error/ATE/RPE/keyframes/loop
  closures/D-optimality, from `/eval/markers`), pose-graph edges, and covariance
  ellipsoids — plus a second map built from the exact simulator pose (`/gt/...`
  topics) overlaid against the SLAM-estimate map, so you can see where the
  belief map diverges from reality, not just how far the path has drifted.

## Running pieces individually

The stack is directly launchable in parts — see
[`slam/stonefish_groundtruth_mapping/launch/`](../slam/stonefish_groundtruth_mapping/launch)
(`tf` → `pointcloud` → `octomap`/`tsdf`, each including the one before it, so
launch only the level you need) and
[`slam/slam_backend/launch/`](../slam/slam_backend/launch) (`sensors_only` for
just the simulated pressure/IMU/DVL sensors, `slam` adds the pose graph on top).

Override the scenario path with `export STONEFISH_WORLD_DIR=/path/to/sim/world`
if not running from the default checkout location.
