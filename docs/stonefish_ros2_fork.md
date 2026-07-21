# stonefish_ros2 — fork provenance

`sim/stonefish_ros2` is a patched fork of the upstream ROS 2 bridge, pinned as
a git submodule. This file records what the fork changes and why, the same way
[`sim/stonefish_patches/README.md`](../sim/stonefish_patches/README.md) does for
the Stonefish C++ library.

**Upstream:** `https://github.com/patrykcieslak/stonefish_ros2.git`
**Base commit:** `6646e7ac25eed982f37f807abc7b545dbd2d6648` (upstream `master`)
**Fork:** `https://github.com/Arcod7/stonefish_ros2.git`, branch `dev`
**Licence:** GPL-3.0, unchanged. `package.xml` deliberately keeps upstream's
maintainer metadata.

## Changes over upstream

1. **Camera timestamps taken on the physics thread.** Vision and sonar sensors
   were stamped on the GL thread, so GPU render latency skewed the association
   between a frame and the pose it was captured at. This underpins every
   ATE/RPE number. Counterpart to Stonefish patch `0002`.

2. **Zero depth pixels replaced with NaN (REP 118).** The depth camera reported
   "no return" as `0.0`, which `depth_image_proc` reads as a point at the
   sensor origin — a phantom blob at the camera that poisons the map. REP 118
   says unmeasured pixels are NaN.

3. **Vertical FoV plumbed through for the depth camera.** The ROS-side half of
   the wide-FoV sonar proxy; pairs with Stonefish patch `0001`, which adds the
   attribute to the sensor and the scenario parser.

4. **`<depend>pcl</depend>` becomes `<depend>libpcl-all-dev</depend>`**, because
   `pcl` is not a rosdep key and dependency resolution failed on a clean
   machine. Same commit adds **`-Wno-psabi` under GCC**, silencing the ABI notes
   aarch64 GCC 10+ emits for `std::pair<float,float>` parameter passing from
   Stonefish's headers — informational only, no behaviour change.

5. **RViz trackball glued to `bluerov2` on startup.** The view defaulted to a
   free camera pointed at nothing in particular; it now follows the vehicle.

## This submodule is required, not optional

`sim/stonefish_ros2` is a colcon package that `stonefish_groundtruth_mapping`
depends on, and it in turn does `find_package(Stonefish REQUIRED 1.6.0)` against
the library in [`external/stonefish`](../external/stonefish). Neither submodule
is optional: a clone without them produces a workspace that fails to build, in
one case with a missing-package error and in the other at compile time.

Always clone with `--recurse-submodules`, or run
`git submodule update --init --recursive` afterwards. `./bootstrap.sh` does this
for you.

## Updating the pin

```bash
cd sim/stonefish_ros2
git fetch origin && git checkout <new-sha>
cd -
git add sim/stonefish_ros2          # records the new gitlink
git submodule status                # no leading '+' means the pin is recorded
```

The `git add` is not optional. `git submodule add` and a bare `git checkout`
inside a submodule leave the superproject still pointing at the old commit, and
`git submodule update` will silently reset the working tree back to it.
