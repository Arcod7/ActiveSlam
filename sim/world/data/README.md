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

**Provenance:** not documented yet — TODO (Antoine): note where each mesh
came from (stock Stonefish/BlueROV2 example assets vs. downloaded third-party
models vs. custom) so a fresh clone knows where to re-fetch them from.

**Getting the files onto a new machine:** until a fetch script exists, copy
`data/obj/` from an existing checkout (or wherever the canonical copies are
kept) into this directory before building.
