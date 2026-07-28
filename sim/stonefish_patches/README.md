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
5. `0005-feat-chase-cam-trackball-can-follow-entity-orientati.patch` — adds an
   opt-in `followOrientation` argument to `OpenGLTrackball::GlueToMoving()`. The
   glued GUI camera previously tracked only the entity's position, keeping a
   fixed world orientation however the vehicle pitched/rolled/yawed; the flag
   layers the entity's per-frame orientation delta onto the current view, giving
   a third-person chase camera that turns with the robot. Zoom (orbit radius,
   mouse scroll) and manual orbiting still work. **Required** —
   `stonefish_ros2` calls the two-argument overload.
6. `0006-fix-apply-mesh-scale-to-OBJ-files-that-carry-no-norm.patch` — makes
   `<mesh scale="...">` take effect on OBJ files that ship no `vn` records.
   `LoadOBJ` writes scaled positions up front, then its no-normals branch
   `memcpy`'d the raw positions back over them, discarding the scale; such a
   mesh always loaded at its native size. Another consequence of upstream's
   move to `rapidobj`. **Required** for `obj_mesh:=shipwreck.obj` — that is the
   one mesh in `sim/world/data/obj` exporting UVs but no normals, so it was the
   only one whose `obj_scale` was ignored.

## Building

```bash
git clone https://github.com/patrykcieslak/stonefish.git
cd stonefish
git checkout b21eb8e194c570ff2f61e91aeffb38d73dc25f42
git am /path/to/ActiveSlam/sim/stonefish_patches/*.patch
# then follow Stonefish's own build instructions (CMake + its 3rdparty deps)
```

`bootstrap.sh` does this for you against `external/stonefish`, which is the
checkout it builds — patch that one, not another clone of the fork.

Building is not enough: `stonefish_simulator` resolves `libStonefish.so` to the
install prefix, so a patch only takes effect after `sudo cmake --install`
(`sudo ldconfig` too, when installing over an older copy). Nothing needs
recompiling in the ROS 2 workspace — the bridge links the library by path — but
the simulator has to be restarted.

`bootstrap.sh` fingerprints the sources it built into
`$prefix/share/Stonefish/.source-id` and rebuilds when that stops matching. It
also probes the installed headers for the patched API, but on its own that test
cannot see a patch which changes no header — 0003, 0004 and 0006 are all
implementation-only.
