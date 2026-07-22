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
updated `./bootstrap.sh`: its guest setup moves only the affected
`libnvidia-container`, `nvidia-container-runtime`, and `nvidia-docker` source
files to `.disabled-by-activeslam*` backups before retrying apt. It does not
disable apt signature verification or delete the source definitions.

**`gtsam==4.2.1 has no wheels with a matching Python implementation tag`
(for example, `cp310`)**

The virtualenv was created from an unsupported system Python. A virtualenv
isolates installed packages, but it retains its base interpreter and version;
one created from Python 3.10 is still Python 3.10. GTSAM 4.2.1 has wheels for
CPython 3.11 and newer, and the supported ROS 2 Jazzy / Ubuntu 24.04
environment supplies Python 3.12.

Check the shell before running bootstrap:

```bash
echo "$ROS_DISTRO"       # jazzy
/usr/bin/python3.12 --version  # Python 3.12.x
```

If those are correct but `<workspace>/.venv/bin/python --version` still says
3.10, deactivate and move or remove the stale venv, then re-run
`./bootstrap.sh`. It will recreate the venv from `/usr/bin/python3.12`.

**`Could not find a package configuration file provided by "Stonefish"`**

`stonefish_ros2` does `find_package(Stonefish)`, which resolves only against an
*installed* `StonefishConfig.cmake` — building the library is not enough. Run
`./bootstrap.sh`; it builds and installs it. If Stonefish is installed
somewhere `bootstrap.sh` does not look but CMake does, pass `--skip-stonefish`.

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

`sim/world/data/obj/` is gitignored and distributed out of band, so a fresh
clone does not have it. `./bootstrap.sh --meshes-from <existing-checkout>/sim/world/data/obj`
copies it and verifies every file against `sim/world/data/obj.sha256`. Running
bootstrap prints exactly which files are missing or corrupted.

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

Expected: neither publishes a wheel this project can use (see
[`INSTALL.md`](INSTALL.md#optional-components)). Build from the pinned
submodules with `./bootstrap.sh --with-vdbfusion` / `--with-open3d`.

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
