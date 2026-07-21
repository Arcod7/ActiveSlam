# Mesh assets (not tracked in git)

`data/obj/` (~313 MB) is gitignored — GitHub rejects pushes containing files
over 100 MB, and the full set is well past that. The directory must exist on
disk with these files for the demo to run; it just never enters git history.

| File | Size | Notes |
|---|---|---|
| `shipwreck.obj` | 289 MB | Wreck structure, main exploration target |
| `bluerov2.obj` | 10 MB | Visual mesh for the BlueROV2 |
| `br2.obj` | 7.3 MB | |
| `cliff.obj` | 7.9 MB | |
| `bluerov2_wings.obj` | 6.6 MB | |
| `off_shore_station.obj` | 5.4 MB | |
| `bluerov2_phy.obj` / `bluerov2_phy.mtl` | 232 KB | Physics/collision mesh |
| `bluerov2_ring.obj` | 17 KB | |
| `ccw.obj` / `cw.obj` | 429–452 KB | |

## BlueROV2 texture atlas

`texture/br2.png` is paired with `obj/bluerov2.obj`. Keep its orientation
unchanged: rotating the atlas by 90 degrees produces apparently random black,
grey and cyan patches because the mesh then samples unrelated UV islands.

**BlueROV2 provenance:** `bluerov2*.obj`, `br2.obj`, `br2.png`, `cw.obj`, and
`ccw.obj` come from the Apache-2.0 licensed
[`bvibhav/stonefish_bluerov2` asset directory](https://github.com/bvibhav/stonefish_bluerov2/tree/master/data/bluerov2).
The environment meshes (`shipwreck.obj`, `cliff.obj`, and
`off_shore_station.obj`) still need their original sources documented.

**Getting the files onto a new machine:** until a fetch script exists, copy
`data/obj/` from an existing checkout (or wherever the canonical copies are
kept) into this directory before building.
