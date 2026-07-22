# Mesh assets

`data/obj/` is split. The BlueROV2 meshes are tracked in git; the rest are
distributed out of band, either because they are too large for GitHub's 100 MB
file limit or because their provenance is not established.

## Tracked in git (~18 MB)

Apache-2.0, from the
[`bvibhav/stonefish_bluerov2` asset directory](https://github.com/bvibhav/stonefish_bluerov2/tree/master/data/bluerov2),
along with `texture/br2.png`.

| File | Size | Loaded by |
|---|---|---|
| `bluerov2.obj` | 10 MB | `data/robot/bluerov2_unphy.scn` |
| `bluerov2_wings.obj` | 6.6 MB | `data/robot/bluerov2_unphy.scn` |
| `ccw.obj` / `cw.obj` | 429–452 KB | both robot scenarios (thruster propellers) |
| `bluerov2_phy.obj` / `bluerov2_phy.mtl` | 232 KB | `data/robot/bluerov2_unphy.scn` |
| `bluerov2_ring.obj` | 17 KB | `data/robot/bluerov2_unphy.scn` |

## Out of band

| File | Size | Loaded by | Why not tracked |
|---|---|---|---|
| `off_shore_station.obj` | 5.4 MB | `scenario/waterlinked.scn` | No recorded source or licence |
| `shipwreck.obj` | 289 MB | nothing | Past GitHub's 100 MB file limit |
| `cliff.obj` | 7.9 MB | nothing | No recorded source or licence |
| `br2.obj` | 7.3 MB | nothing | Apache-2.0, but unreferenced |

`off_shore_station.obj` is the only one a scenario actually loads, so it is the
only one `bootstrap.sh` asks for. The other three are unreferenced: a checkout
without them is complete, and the simulator never opens them.

**Publishing `off_shore_station.obj` is blocked on provenance.** It is the
environment the demo runs in, so tracking it would make the repo entirely
self-contained — but redistributing third-party geometry under an unknown
licence is not something to do by default. Settle the source first.

## BlueROV2 texture atlas

`texture/br2.png` is paired with `obj/bluerov2.obj`. Keep its orientation
unchanged: rotating the atlas by 90 degrees produces apparently random black,
grey and cyan patches because the mesh then samples unrelated UV islands.

## Verifying a copy

[`obj.sha256`](obj.sha256) records the SHA-256 of the out-of-band meshes — git
already guarantees the tracked ones. A checkout can tell whether its copy is
intact:

```bash
cd sim/world/data && sha256sum -c obj.sha256
```

Run directly, that reports the unreferenced meshes as failures when they are
simply absent. `./bootstrap.sh` hashes only the files on disk and asks for a
missing mesh only when a scenario loads it.

## Getting the files onto a new machine

Copy the out-of-band files from an existing checkout —
`./bootstrap.sh --meshes-from <path>` does this and then verifies them.

Hosting them somewhere fetchable remains an open decision, still blocked on the
provenance question above. Were it settled, a GitHub release asset is the
option to pick: no extra tooling, no repo bloat, works for anonymous clones,
versioned alongside the code, and well inside the 2 GB per-asset limit. git-lfs
bloats every clone by default and the free quota (1 GB storage, 1 GB/month
bandwidth) is smaller than the asset set; an external institutional URL goes
dead and takes the repo's buildability with it.
