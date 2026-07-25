# Troubleshooting

Failure modes seen on real installs, and what each one actually means.
Install steps are in [`INSTALL.md`](INSTALL.md).

## Build

**Bootstrap reports an unsupported Ubuntu 22.04 / ROS 2 Humble environment**

Accept the interactive Distrobox prompt. It uses Podman and Distrobox's
official curl installer, creates an Ubuntu 24.04 container named
`activeslam-jazzy`, enables NVIDIA integration when the host driver is
detected, installs ROS 2 Jazzy, and resumes bootstrap inside it. Later, enter
the same environment with:

```bash
distrobox enter activeslam-jazzy
source /opt/ros/jazzy/setup.bash
```

If the failed Humble attempt already created a Python 3.10 `.venv`, the
Distrobox handoff moves it to a timestamped `.venv.pre-jazzy-*` backup before
creating the Python 3.12 venv.

The prompt appears only on a terminal; non-interactive invocations fail rather
than installing host software unexpectedly.

**`NO_PUBKEY DDCAE044F796ECB0` for an `nvidia.github.io` repository inside
the Distrobox**

Legacy NVIDIA Container Toolkit apt sources from an Ubuntu 22.04 host can be
reproduced in the Ubuntu 24.04 guest without their old signing key. They are
not required for Distrobox's `--nvidia` driver-library integration. Re-run the
updated `./bootstrap.sh`: its guest setup gives apt a container-local filtered
view containing all legitimate Ubuntu and ROS sources except the affected
`libnvidia-container`, `nvidia-container-runtime`, and `nvidia-docker` feeds.
It does not rename, edit, or delete the host-mounted files, and it does not
disable apt signature verification.

**`gtsam==4.2.1 has no wheels with a matching Python implementation tag`
(for example, `cp310`)**

The virtualenv was created from an unsupported system Python. A virtualenv
isolates installed packages, but it retains its base interpreter and version;
one created from Python 3.9 is still Python 3.9. The pinned GTSAM wheel
(`GTSAM_WHEEL_VERSION` in `dependencies.conf`) covers CPython 3.10 for Humble
and 3.12 for Jazzy, which are what those distributions supply. Anything else
falls back to the source build.

Check the shell before running bootstrap:

```bash
echo "$ROS_DISTRO"             # jazzy or humble
/usr/bin/python3.12 --version  # Python 3.12.x on Jazzy
/usr/bin/python3.10 --version  # Python 3.10.x on Humble
```

If those are correct but `<workspace>/.venv/bin/python --version` disagrees,
deactivate and move or remove the stale venv, then re-run `./bootstrap.sh`. It
will recreate the venv from the interpreter the distribution expects.

**`Could not find a package configuration file provided by "Stonefish"`**

`stonefish_ros2` does `find_package(Stonefish)`, which resolves only against an
*installed* `StonefishConfig.cmake` — building the library is not enough. Run
`./bootstrap.sh`; it builds and installs it. If Stonefish is installed
somewhere `bootstrap.sh` does not look but CMake does, pass `--skip-stonefish`.

**`sf::Camera has no member named getLastCaptureTime`** (or
`sf::DepthCamera has no member named getVerticalFOV`)

The installed Stonefish predates the patched fork. Both methods were added on
the fork and `stonefish_ros2` calls them, so an older install compiles the
bridge no further than this — while still satisfying
`find_package(Stonefish)`, which is why the build gets as far as it does.
Confirm with:

```bash
grep -c getLastCaptureTime /usr/local/include/Stonefish/sensors/vision/Camera.h
```

`0`, or no such file, means the install is stale. `./bootstrap.sh` detects this
and rebuilds over it. To do it by hand:

```bash
git submodule update --init --recursive external/stonefish
cmake -S external/stonefish -B external/stonefish/build -DCMAKE_BUILD_TYPE=Release
cmake --build external/stonefish/build -j"$(nproc)"
sudo cmake --install external/stonefish/build && sudo ldconfig
```

Note that `--skip-stonefish` suppresses the check along with the build, so an
install kept deliberately out of the way has to stay current by hand.

**vdbfusion's source build fails in c-blosc: `conflicting types for
'shuffle'`**

Seen on aarch64 with GCC 11 (Ubuntu 22.04 / Humble). vdbfusion's CMake builds
its own blosc 1.5.0 through ExternalProject rather than using the system one,
and that release's `shuffle.c` and `shuffle.h` disagree on a `const`, which GCC
11 rejects outright where older compilers only warned.

Nothing in this repository selects that blosc. The combination is narrow —
x86_64 takes the wheel and never builds, and the source build works on Ubuntu
24.04 — so if you hit it, `./bootstrap.sh --skip-vdbfusion` gets you a working
workspace without `mapper:=tsdf`; use `mapper:=octomap`.

**`fatal error: glm/glm.hpp: No such file or directory`**

Stonefish's own dependencies are missing. `rosdep` cannot supply them — it only
reads this workspace's `package.xml` files, and Stonefish is not a ROS package:

```bash
sudo apt install libglm-dev libsdl2-dev libfreetype-dev libgl1-mesa-dev
```

**`ModuleNotFoundError: No module named 'catkin_pkg'`, from a Python that is
not `/usr/bin/python3`**

CMake picked up a non-system interpreter — typically a uv- or pyenv-managed
Python earlier on `PATH` (`~/.local/bin/python3.x`). It lacks ROS 2's Python
packages. The venv was not active. Activate it and rebuild — wiping `build/` first, since
the wrong interpreter is cached in `CMakeCache.txt` and a plain rebuild keeps
using it:

```bash
source <workspace>/.venv/bin/activate
rm -rf build install log
colcon build --symlink-install --cmake-args -Wno-dev
```

**A node dies on `ModuleNotFoundError: No module named 'gtsam'` (or
`small_gicp`, `vdbfusion`), even though the venv has it**

The workspace was built without the venv active. `ament_python` entry points
take their shebang from whichever interpreter ran `colcon`, so the installed
scripts point at a Python that cannot see the venv — and activating it
afterwards does not help, because a shebang bypasses `PATH`. Check with:

```bash
head -1 install/slam_backend/lib/slam_backend/pose_graph
```

If that is not the venv's python, rebuild as above, or re-run `./bootstrap.sh`.

If the shebang *is* right, the package is simply absent — check directly:

```bash
<workspace>/.venv/bin/python -c "import vdbfusion"
```

`./bootstrap.sh` installs vdbfusion by default, but a workspace bootstrapped
before that was the case, or with `--skip-vdbfusion`, will not have it, and
`mapper:=tsdf` exits on import. Re-run `./bootstrap.sh`.

Note that the apt `colcon` has a `/usr/bin/python3` shebang of its own, so it
builds with the system interpreter no matter which venv is active.
`requirements.txt` therefore installs `colcon` *into* the venv, which is what
makes activating it sufficient.

**A renamed or added file in a `setup.py` `data_files` list is not installed
correctly**

`colcon` does not prune installed files when a `data_files` entry changes, so an
incremental `--symlink-install` rebuild can leave a stale symlink beside the new
file. Remove `build/ install/ log/` and build again.

**CMake dev warnings from PCL (`CMP0144`/`CMP0074` about `_ROOT` variables)**

Noise from upstream PCL's own cmake modules, not this project. `bootstrap.sh`
passes `-Wno-dev` to silence it.

## Launch

**`PackageNotFoundError: package 'octomap' not found`**

A declared dependency is not installed. Re-run `rosdep install --from-paths src
-i -y` from the workspace root, or just `./bootstrap.sh`.

**Stonefish exits complaining about the scenario, or meshes look absent**

The BlueROV2 meshes are tracked in git, but `off_shore_station.obj` is
distributed out of band, so a fresh clone has the robot and no environment.
`./bootstrap.sh --meshes-from <existing-checkout>/sim/world/data/obj` copies it
and verifies it against `sim/world/data/obj.sha256`. Running bootstrap prints
exactly which files are missing or corrupted.

**`Unable to connect to a Zenoh router`, and nodes do not see each other**

`RMW_IMPLEMENTATION` is set to `rmw_zenoh_cpp` in that shell. Unlike the DDS
implementations, it needs a router process running before peers can discover
one another — without it every node starts, publishes into nothing, and the
stack looks up while no data flows. Either start the router in its own
terminal:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

or drop back to the distribution default, which is what this project is
developed against — nothing here selects an RMW:

```bash
unset RMW_IMPLEMENTATION
```

Check what is in effect with `echo "$RMW_IMPLEMENTATION"`; empty is the
default. Note it must match across every terminal involved, including the one
running the launcher.

**RViz: `The plugin for class 'octomap_rviz_plugins/OccupancyGrid' failed to
load`**

`octomap_rviz_plugins` is not installed. `rviz/demo.rviz` uses it, and
`bringup` declares it, so `rosdep install --from-paths src -i -y` — or
`./bootstrap.sh` — pulls it in. A workspace set up before that declaration
existed needs one of those re-run. RViz itself still opens; only the octomap
display is missing.

**`pose_graph` dies with exit code -11 (SIGSEGV) and prints nothing**

A segfault before its first log line means the crash is in `_setup_gtsam` or an
import above it, not in the node's own logic. Usually the installed GTSAM is
not the pinned one — a source build from an earlier run stays importable, and
until this was version-checked, re-running `./bootstrap.sh` kept it. Check what
is actually there:

```bash
<workspace>/.venv/bin/python -c "import importlib.metadata as m; print(m.version('gtsam'))"
```

If that is not `GTSAM_WHEEL_VERSION` from `dependencies.conf` (nor
`GTSAM_SOURCE_REF`, on a platform with no wheel), re-run `./bootstrap.sh`,
which now replaces it. To see the crash itself:

```bash
PYTHONFAULTHANDLER=1 ros2 run slam_backend pose_graph
```

That prints the Python frame it died in — enough to tell an import apart from a
GTSAM call. Note the whole stack depends on this node: it is the only publisher
of `/slam/odometry`, so without it the safety gate reports `NO_ODOMETRY` and
zeroes all motion however you arm it.

**`no other ROS nodes are visible — discovery is not working`**

The launcher could not see a single node while the stack was up, so the problem
is the middleware, not the layer you were trying to reach. The message reports
the `RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID` in effect *for the launcher*;
compare them against a node's:

```bash
tail -n +1 logs/launcher/latest.log | grep middleware   # what the launcher used
echo "$RMW_IMPLEMENTATION $ROS_DOMAIN_ID"               # in any other terminal
```

They must match everywhere, and unsetting `RMW_IMPLEMENTATION` in one shell
does not affect a launcher already running in another — start the launcher from
a shell where it is already right. If your shell profile exports it, unsetting
it interactively is undone the next time anything re-sources the profile.

**Nothing moves, however you drive it**

Working as designed: the motion safety gate is fail-closed and starts disabled.
Arm it from the RViz panel, the launcher's `m` key, or
`safety_start_enabled:=true`. `/motion/safety_status` reports what is blocking —
including `MULTIPLE_COMMAND_SOURCES`, which stops all motion when two publishers
share `/motion/body_command` (e.g. teleop while the frontier planner is live).

**RViz dies with `Invalid parentWindowHandle` / `Unable to create the rendering
window after 100 tries`**

Qt is failing to pick a working platform plugin, seen on Wayland sessions with
some GL drivers. Force X11 in the shell you launch from:

```bash
export QT_QPA_PLATFORM=xcb
```

This belongs in your own shell profile, not in the repo — it depends on the
machine, not the project.

## Python dependencies

**`error: externally-managed-environment` (PEP 668)**

Ubuntu 24.04 refuses a bare `pip install` into the system Python. The project
uses a uv virtualenv at `<workspace>/.venv` for exactly this reason —
`./bootstrap.sh` creates it. Activate it before running anything:
`source <workspace>/.venv/bin/activate`.

**`pip install vdbfusion` or `pip install open3d` finds no matching
distribution**

Expected on aarch64 and on CPython 3.11+ (see
[`INSTALL.md`](INSTALL.md#optional-components)). `./bootstrap.sh` handles
vdbfusion itself, falling back to the pinned submodule when no wheel matches;
Open3D needs `--with-open3d`.

## aarch64 (Apple Silicon, Raspberry Pi)

**`import open3d` raises `cannot allocate memory in static TLS block`**

Raise the loader's static TLS reserve for the process:

```bash
GLIBC_TUNABLES=glibc.rtld.optional_static_tls=2097152
```

This resolves it — verified 2026-07-22: `import open3d` succeeds and
`compute_fpfh_feature` returns a 33xN descriptor matrix on a 2000-point cloud.
A ROS node needing Open3D must have the variable in its environment before the
process starts, so set it from the launch file (`SetEnvironmentVariable` or a
node's `additional_env`), not from inside the module. The trailing
`OpenBLAS : munmap failed` and `error code=22` lines printed at interpreter
exit are harmless allocator complaints on a 16 KB-page kernel, not a failure.

Upgrading Open3D does not avoid this. Its own fix
(`add_compile_options("-ftls-model=global-dynamic")` under `LINUX_AARCH64`) is
identical in v0.19.0 and on `main`, and no commit since has touched static TLS.
The built module needs 12 KB of thread-local storage and carries a single
initial-exec relocation (`R_AARCH64_TLS_TPREL64`) against nine well-behaved
`TLSDESC` ones; that one relocation forces the whole block into the static TLS
area at `dlopen`, and a 16 KB-page kernel leaves less surplus to satisfy it
from. It is not in any of the bundled static libraries.

**VTK's download fails with `SSL connect error` from inside a container that
reaches other hosts fine**

Fetch `https://vtk.org/files/release/9.1/VTK-9.1.0.tar.gz` from the host into
`external/open3d/3rdparty_downloads/vtk/` and re-run. CMake verifies the
SHA-256 and skips the download.
