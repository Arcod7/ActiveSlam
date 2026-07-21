# Stonefish patches

This project runs a patched build of the [Stonefish](https://github.com/patrykcieslak/stonefish)
underwater simulator. The patches are kept here as plain `.patch` files rather
than a vendored fork or submodule, since the base library is a large C++
project (~250 MB checkout) that isn't part of this repo.

**Upstream:** `https://github.com/patrykcieslak/stonefish.git`
**Base commit:** `b21eb8e194c570ff2f61e91aeffb38d73dc25f42`
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
3. `0003-fix-use-GL_LINEAR-for-texture-magnification-filter.patch` — corrects an
   invalid OpenGL enum: `GL_TEXTURE_MAG_FILTER` was set to
   `GL_LINEAR_MIPMAP_LINEAR`, which only `GL_TEXTURE_MIN_FILTER` accepts. The
   call was a silent no-op (raising `GL_INVALID_ENUM`, leaving the filter at its
   `GL_LINEAR` default); this makes it explicit. Independent upstream cleanup,
   not something the project relies on.
4. `0004-fix-do-not-treat-a-missing-material-library-as-fatal.patch` — stops a
   missing `*.mtl` from aborting the simulator. Since upstream moved OBJ loading
   to `rapidobj`, `ParseFile` resolves the `mtllib` directive with
   `Load::Mandatory`, so a mesh whose material library was never shipped kills
   the process through `cCritical`. Several of this project's meshes are Blender
   exports carrying such a dangling reference, and `LoadOBJ` never reads the
   parsed materials anyway (Stonefish takes them from the scenario XML), so the
   parse now uses `MaterialLibrary::Ignore()`. **Required** — without it the
   simulator aborts while parsing the scenario.

## Building

```bash
git clone https://github.com/patrykcieslak/stonefish.git
cd stonefish
git checkout b21eb8e194c570ff2f61e91aeffb38d73dc25f42
git am /path/to/ActiveSlam/sim/stonefish_patches/*.patch
# then follow Stonefish's own build instructions (CMake + its 3rdparty deps)
```
