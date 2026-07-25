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
- **drives the vehicle straight from the option screen** — the drive keys are
  always live, there is no teleop mode to enter. `W/S` forward/back, `Q/E`
  strafe, `A/D` yaw, `Space/X` up/down, `F` stop (QWERTY; an AZERTY option
  under *Keyboard layout* maps the same physical keys). The commands match
  `ros2 run launch_tools my_keyboard`, so `mode:=teleop` does not need a
  second terminal. Pressing a drive key in frontier mode suspends the planner
  first — the safety gate treats two publishers on `/motion/body_command` as
  `MULTIPLE_COMMAND_SOURCES` and stops all motion — and `Esc` hands control
  back to autonomy.
- **swims to a target point** — the *Mode* option's third value, `goto`. The
  same drive keys move a green RViz marker instead of the vehicle, along world
  axes (`W/S` X, `A/D` Y, `Q/E` Z; `F` recalls the point to the vehicle — a
  point has no heading, so there are no yaw keys), and the planner paths to it:
  the point is published on `/frontier_slam/goal` with frontier goal picking
  suspended, so A* and the path executor drive there exactly as they would to
  a frontier. `goto` runs the planner layer without `revisit`, so nothing
  preempts the operator's goal.
- **retunes wall-following parameters live**, via `ros2 param set`, with no
  restart at all — those parameters are re-read every control cycle. Options
  that support this are marked `(live)`.
- **starts the viewers.** RViz is on by default; the *rqt* option adds an
  `rqt` layer beside it (node graph, topic monitor, plots, parameter
  reconfigure) for when a run needs introspection rather than a demo view.
- **shuts down cleanly.** Every layer runs in its own process group and is torn
  down with an escalating SIGINT → SIGTERM → SIGKILL, on quit, on Ctrl-C, and
  on exit, so nothing is left running in the background.

### Logs

Everything lands in `logs/launcher/` (gitignored):

| File | What it holds |
|---|---|
| `latest.log` | Symlink to the current run's `session-<timestamp>.log` |
| `session-<timestamp>.log` | One run, merged and timestamped: launcher events, every layer's output labelled by group, and anything the middleware writes to stderr |
| `<group>.log` | One layer's raw output, appended to across runs |

Tail the merged one to watch a run:

```bash
tail -f logs/launcher/latest.log
```

That is the file to read when something dies mid-run. A layer going down on its
own is recorded as it happens — `core: running -> exited (exit codes: [1])` —
which a screen showing only current state cannot tell you. It is also where
middleware warnings go: they are written straight to file descriptor 2, so
without this they print on top of the TUI and corrupt the display.

Selections persist in `~/.activeslam_launcher.json`, so it reopens on the last
configuration used.

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
ros2 launch bringup demo.launch.py mapper:=tsdf tsdf_octomap:=false  # skip the TSDF→OcTree bridge
ros2 launch bringup demo.launch.py noise_profile:=realistic|ideal|sonar_only|odom_pos_only|odom_only|degraded  # slam:=slam only
ros2 launch bringup demo.launch.py mode:=frontier scan_style:=sweep|spin \
  scan_sweep_deg:=180.0                                           # cable-safe sweep (default) vs full revolution
ros2 launch bringup demo.launch.py mode:=frontier safety_start_enabled:=true  # arm the motion gate at startup
ros2 launch bringup demo.launch.py mode:=frontier motion:=walloriented \
  wall_orientation_offset_deg:=30                                 # follow path, look toward nearest wall
```

`mapper:=tsdf` needs vdbfusion, which `./bootstrap.sh` installs by default.

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
ros2 launch bringup demo.launch.py slam:=slam noise_seed:=7                   # repeatable, decorrelated sensor-noise draws
ros2 launch bringup demo.launch.py slam:=slam output_dir:=/path/to/run        # label eval output instead of a timestamp
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier revisit:=true    # break off exploration to close loops when uncertainty grows
```

The TSDF mapper preserves capture-time geometry when cloud and TF messages arrive
out of order: it waits asynchronously for the exact timestamp in a bounded FIFO
(defaults: 10 clouds, 0.5 s) rather than substituting the latest pose. Its periodic
`Cloud/TF stats` log reports received, deferred, recovered, expired, failed, and
overflow counts; sustained expiry/overflow indicates TF or processing overload.

A TSDF rebuild is triggered by a large pose-graph correction and therefore often
appears at the same time as a position-error change, but the mapper does not write
SLAM pose. To attribute a regression, align `metrics.csv` with the `Loop closure`,
`Map rebuild starting`, and `Map rebuild replay complete` timestamps in
`launch.log`. If error jumps before rebuild start, investigate closure acceptance;
if a fixed-duration run lacks `replay complete`, extend or invalidate that cell
before comparing map quality. Phase 39 contains a reproduced wall-oriented case
where error jumped 139 ms before rebuild start.

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

`noise_seed` controls the sensor RNG streams; it does not serialize Stonefish,
ROS callbacks, map updates, or planner decisions. Repeating an identical seed can
therefore produce a different path and different loop closures. Use a multi-seed
matrix for comparisons rather than treating one seeded run as an exact replay.

`metrics.csv` scores the continuously corrected `/slam/odometry` stream. ATE is
therefore approximately time-uniform, and RPE uses a fixed one-second separation
by default (`eval.launch.py rpe_delta:=<seconds>`). Matrix `status` is only a
structural-validity result (data present, real motion, adequate duration, no
traceback/process death); it is not an accuracy pass/fail threshold.

## RViz views

The view switches automatically with `mapper`/`slam` (`slam:=slam` wins if both
apply — see [`bringup/rviz/`](../bringup/rviz)):

- default (`mapper:=octomap slam:=none`): the base view.
- `mapper:=tsdf`: the TSDF surface plus `OcTree (occupied)`. `octomap_server`
  is not launched in this mode — `/octomap_binary` comes from
  `tsdf_to_octomap`, which rebuilds an `octomap::OcTree` out of the TSDF's
  occupied and free voxels (`tsdf_octomap:=true`, the default). With the
  bridge off, enable the marker-based `TSDFVoxels` display instead.
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
