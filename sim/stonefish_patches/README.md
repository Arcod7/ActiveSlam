# Stonefish patches

This project runs a patched build of the [Stonefish](https://github.com/patrykcieslak/stonefish)
underwater simulator. The patches are kept here as plain `.patch` files rather
than a vendored fork or submodule, since the base library is a large C++
project (~250 MB checkout) that isn't part of this repo.

**Upstream:** `https://github.com/patrykcieslak/stonefish.git`
**Base commit:** `09208f913daf688cc8abc775d45742c860c81f19`
**Patches (apply in order):**
1. `0001-feat-set-vertical-fov-for-depth-camera.patch` — adds a `vertical_fov`
   attribute to depth-camera sensors (XML + `DepthCamera` class), so a depth
   camera can be configured with an independent elevation aperture. This is
   how the project emulates a wide-FoV 3D sonar (the OpenGL projection layer
   already supported this; the patch plumbs it up to the sensor and scenario
   parser).
2. `0002-feat-snapshot-camera-timestamps-on-the-physics-threa.patch` — records
   capture timestamps on the physics thread (not the GL thread) for every
   vision/sonar sensor, so ground-truth pose association isn't skewed by GPU
   render latency.

## Building

```bash
git clone https://github.com/patrykcieslak/stonefish.git
cd stonefish
git checkout 09208f913daf688cc8abc775d45742c860c81f19
git am /path/to/ActiveSlam/sim/stonefish_patches/*.patch
# then follow Stonefish's own build instructions (CMake + its 3rdparty deps)
```
