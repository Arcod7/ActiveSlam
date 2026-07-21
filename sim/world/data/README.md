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

## Verifying a copy

[`obj.sha256`](obj.sha256) records the SHA-256 of every file `obj/` must
contain. It is tracked in git even though the meshes are not, so a checkout can
always tell whether its copy is complete and intact:

```bash
cd sim/world/data && sha256sum -c obj.sha256
```

`./bootstrap.sh` runs this and names any file that is missing or corrupted.
Without it a partial copy surfaces much later, as an obscure failure inside the
simulator.

## Getting the files onto a new machine

Copy `obj/` from an existing checkout — `./bootstrap.sh --meshes-from <path>`
does this and then verifies against the manifest.

**Automating this is still an open decision**, and it is blocked on two
separate things:

1. *Where to host them.* The options, with their trade-offs:
   - **GitHub release asset** — a tarball attached to a release. No extra
     tooling, no repo bloat, works for anonymous clones, and versioned
     alongside the code. 313 MB is within GitHub's 2 GB per-asset limit. This
     is the option to pick unless something rules it out.
   - **git-lfs** — cleanest conceptually, but it bloats every clone by
     default, needs LFS installed, and GitHub's free LFS quota (1 GB storage,
     1 GB/month bandwidth) is smaller than this asset set.
   - **External URL** (institutional storage) — fine for the project's own
     use, but a dead link makes the repo unbuildable for anyone else later.
2. *Provenance.* `shipwreck.obj`, `cliff.obj` and `off_shore_station.obj` have
   no recorded source or licence (see above). Redistributing them without
   knowing their licence is not something to do by default, so that has to be
   settled before any of them are published anywhere — regardless of which
   hosting option is chosen.

The BlueROV2 assets are unaffected by (2): they are Apache-2.0 and already
attributed above, so they could be published immediately if the environment
meshes were split out.
