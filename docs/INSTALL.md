# Install

The short version is in the root [`README.md`](../README.md). This page covers
the prerequisites it assumes, and what each step of `bootstrap.sh` actually
does. When something fails, see [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md).

## Prerequisites

**ROS 2 Jazzy**, which targets Ubuntu 24.04. Follow the official installation
guide and pick the `desktop` variant — it includes RViz, which this project
uses. On another Linux distro, a container tool like
[distrobox](https://github.com/89luca89/distrobox) is one way to get an
Ubuntu 24.04 userspace. When `bootstrap.sh` is run interactively from an
unsupported environment (for example, Ubuntu 22.04 with ROS 2 Humble), it
offers to install Podman and Distrobox, create an `activeslam-jazzy` Ubuntu
24.04 container, install ROS 2 Jazzy there, and resume automatically. NVIDIA
integration is enabled when the host driver is detected. Declining the prompt,
or running non-interactively, leaves the host unchanged and exits.

**`rosdep`**, which is not always installed alongside ROS 2 itself:

```bash
sudo apt install python3-rosdep
sudo rosdep init   # only if rosdep has never been set up on this machine
rosdep update
```

**The scene mesh.** The BlueROV2 meshes are tracked in git, but
`off_shore_station.obj` — the environment `scenario/waterlinked.scn` loads — is
distributed out of band, as its provenance is unrecorded. It must be on disk
before the simulator can load the scenario. `bootstrap.sh --meshes-from <path>`
copies it from an existing checkout and verifies it against a checksum
manifest. See [`sim/world/data/README.md`](../sim/world/data/README.md).

Everything else — build tools, Python packages, Stonefish's dependencies — is
installed by `bootstrap.sh`. The Python packages go into a virtualenv, but the
Python interpreter itself still comes from the supported Ubuntu 24.04
environment: a virtualenv isolates packages; it does not change Python 3.10
into Python 3.12.

## What `bootstrap.sh` does

Run it from the repo root. In the supported environment, `ros2` must be on
`PATH`; from an interactive unsupported host, the Distrobox prompt can create
that environment first. The script is idempotent: re-run it after a `git pull`,
or whenever a step failed and you have fixed the cause.

1. **Sanity checks** — requires ROS 2 Jazzy, `rosdep`, and Python 3.12. In an
   interactive unsupported shell, offers the Distrobox handoff described
   above before exiting.
2. **Submodules** — initialises the two required ones, `external/stonefish`
   (the patched library) and `sim/stonefish_ros2` (the patched ROS 2 bridge).
   The workspace does not build without them, so cloning with
   `--recurse-submodules` is unnecessary; this step covers it either way.
   `external/vdbfusion` and `external/open3d` are optional and are fetched only
   by `--with-vdbfusion` / `--with-open3d`, which keeps ~350 MB of Open3D out
   of a default clone.
3. **Python dependencies** — Ubuntu 24.04 marks its system Python
   externally-managed, so a bare `pip install` fails with PEP 668. They go into
   a [uv](https://docs.astral.sh/uv/) virtualenv at `<workspace>/.venv` instead,
   created with `--system-site-packages` so the distro's `rclpy` and the rest of
   ROS 2's Python stay visible. `uv` itself is installed if missing, and an
   already-active virtualenv is reused rather than replaced.
   [`requirements.txt`](../requirements.txt) holds only what `rosdep` cannot
   provide; `numpy`, `scipy` and `matplotlib` are declared in the `package.xml`
   files instead and come from apt.
4. **`rosdep`** — resolves every dependency declared across the workspace's
   `package.xml` files (`depth_image_proc`, `octomap_server`, `octomap`,
   `rviz2`, ...).
5. **Patched Stonefish** — installs its dependencies (glm, SDL2, Freetype,
   OpenGL, which `rosdep` does not cover), then builds and installs the pinned
   submodule. Skipped when `StonefishConfig.cmake` is already present under a
   standard prefix; `--skip-stonefish` forces the skip. Building takes a while.
6. **vdbfusion**, only with `--with-vdbfusion` — needed for `mapper:=tsdf`.
7. **Open3D**, only with `--with-open3d` — needed for FPFH descriptor work.
8. **Scene meshes** — copies the out-of-band ones with `--meshes-from`, then
   hashes whatever is on disk against `sim/world/data/obj.sha256`. A missing or
   corrupted mesh a scenario needs is reported here rather than failing later
   inside the simulator. This is a warning, not a failure: the rest of the
   build still completes.
9. **`colcon build --symlink-install`** over the whole workspace, with the venv
   on `PATH` — which is all that activating it does. Rebuild the same way:

   ```bash
   source <workspace>/.venv/bin/activate
   colcon build --symlink-install --cmake-args -Wno-dev
   ```

   Building without the venv active produces a workspace whose nodes cannot
   import `gtsam` and the other wheels — see
   [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) for why. `requirements.txt`
   installs `colcon` into the venv so that activating it is genuinely enough.

Then activate the virtualenv and source the workspace, in each new shell:

```bash
source <workspace>/.venv/bin/activate
source <workspace>/install/setup.zsh
```

## Optional components

Both are pinned as submodules because neither publishes a wheel this project
can use, and both are source builds you can skip entirely.

**vdbfusion** (`mapper:=tsdf`) — PyPI only publishes wheels up to Python 3.10,
x86_64 only, so `pip install vdbfusion` fails on a fresh Jazzy setup (Python
3.12) on any architecture.

```bash
./bootstrap.sh --with-vdbfusion
```

It needs OpenVDB and a C++ toolchain, which are not part of this project's
dependency list — follow
[vdbfusion's own `INSTALL.md`](https://github.com/PRBonn/vdbfusion/blob/main/INSTALL.md)
if the build fails.

**Open3D** (FPFH submap descriptors, point-cloud registration) — no wheel is
published for aarch64 on any Python version, so on Apple Silicon or a Raspberry
Pi a source build is the only option. Pinned at `v0.19.0`.

```bash
./bootstrap.sh --with-open3d
```

The build is heavy: it compiles Open3D's own third-party dependencies,
including VTK from source. It is configured with CUDA, the GUI, examples and
unit tests off, since none are used here.

## Patched forks

Stonefish is a patched build, not the stock package.
[`sim/stonefish_patches/`](../sim/stonefish_patches/) carries the patches as
plain files and records the upstream base commit — they document what the fork
changes; the pinned submodule is what actually gets built. The ROS 2 bridge is a
patched fork too — see [`stonefish_ros2_fork.md`](stonefish_ros2_fork.md).

## Verify

```bash
ros2 launch bringup demo.launch.py
```

Should bring up Stonefish and RViz, and print a reminder to run
`ros2 run launch_tools my_keyboard` in another terminal (the default mode is
`teleop`). See [`RUN.md`](RUN.md) for the full set of switches.
