# genesis-sand-water-walker

Soft-terrain locomotion experiments built on top of
[Genesis](https://github.com/Genesis-Embodied-AI/Genesis): we drop a humanoid
into a sand pool and a planar walker into a water pool, using Genesis's
MPM ↔ rigid-body coupling.

The long-term goal is **stepping/walking on sand and water** — a learned policy
that keeps a bipedal robot upright while interacting with a deformable medium.
The dive scripts in this repository are the baseline scene/coupling setup that
goal will build on; no controller is trained yet.

## Demos

The two scripts below each produce an MP4 of the dive plus a per-step CSV of
the torso trajectory. Output goes to `outputs/`.

| Scenario | Script | Output |
| --- | --- | --- |
| Humanoid → 0.75 m deep sand, 3.6 m drop | `scripts/humanoid_on_sand.py` | `outputs/humanoid_on_sand.{mp4,csv}` |
| Planar walker → 0.75 m deep water, 3.2 m drop | `scripts/walker_on_water.py` | `outputs/walker_on_water.{mp4,csv}` |

## Requirements

- Linux host with an NVIDIA GPU (tested on RTX 5080)
- Docker with the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
  (i.e. `docker run --gpus all` must work)
- `make`

All Python and CUDA dependencies live inside the Docker image, so no host-side
Python setup is required.

## Quick start

```bash
make help                # list every target
make build               # build the Docker image (one-time, ~30 min: it compiles LuisaRender)
make check-gpu           # optional: confirm Docker can see the GPU
make dive-humanoid       # humanoid dive into sand → outputs/humanoid_on_sand.mp4
make dive-walker         # walker dive into water → outputs/walker_on_water.mp4
make dive-all            # both, sequentially
```

Each dive simulates 2.4 s of physics (600 steps, `dt = 4 ms`, 25 sub-steps for
MPM ↔ rigid stability) and takes roughly **5–7 minutes** of wall time on an
RTX 5080. The bottleneck is the MPM solver, not rendering.

`make shell` drops you into an interactive bash inside the same container if
you want to iterate on the scripts without rebuilding.

## Repository layout

```
genesis-sand-water-walker/
├── Makefile               # all entrypoints — `make help` lists them
├── docker/
│   ├── Dockerfile         # CUDA 12.8 + PyTorch 2.11 + Genesis + LuisaRender
│   ├── build_luisa.sh
│   └── *.json             # NVIDIA EGL/Vulkan ICD descriptors
├── scripts/
│   ├── humanoid_on_sand.py
│   └── walker_on_water.py
├── assets/
│   ├── humanoid_no_floor.xml   # MJCF copies of Genesis's bundled models
│   └── walker_no_floor.xml     # with the worldbody floor plane removed
└── outputs/               # generated videos + CSVs (gitignored)
```

## Why the custom MJCFs

`assets/humanoid_no_floor.xml` and `assets/walker_no_floor.xml` are derived
from Genesis's stock `genesis/assets/xml/humanoid.xml` and `walker.xml` with
the `<geom name="floor">` plane removed from `<worldbody>`.

That floor plane is part of the parsed MJCF entity. When you load the original
file with an initial-pose offset (`gs.morphs.MJCF(file=..., pos=(0,0,h))`),
the offset translates **every** link in the entity — including the floor —
so the robot ends up standing on its own internal floor and never falls.
Removing the plane lets gravity actually act on the body.

If you want to verify this for yourself, set the script to use the stock
MJCF path: the torso z-coordinate will stay pinned and the sand/water below
will be undisturbed.

## Simulation knobs

The interesting parameters live near the top of each script:

| Knob | Where | Meaning |
| --- | --- | --- |
| `sim_options.dt`, `substeps` | `gs.options.SimOptions(...)` | Rigid-body integrator time-step. 25 substeps is enough to keep contact impulses stable for a 3 m drop into deep MPM media; lower it if you see `Invalid constraint forces causing 'nan'`. |
| `mpm_options.grid_density` | `gs.options.MPMOptions(...)` | MPM grid resolution (cells per unit length). 40 is a balance between visual quality and runtime. |
| Pool depth | `gs.morphs.Box(size=(..., ..., depth), ...)` | Currently 0.75 m, ≈ half the humanoid's height. |
| Drop height | `gs.morphs.MJCF(..., pos=(0, 0, h))` | 2.30 m for humanoid (torso lands from ~3.6 m), 1.90 m for walker. |
| `needs_coup`, `coup_friction` | `gs.materials.Rigid(...)` | MPM ↔ rigid coupling. Disable `needs_coup` and the robot will pass straight through the sand/water. |

## Roadmap

- [x] Drop a humanoid into a deep sand pool (scene + coupling validated)
- [x] Drop a planar walker into a deep water pool (scene + coupling validated)
- [ ] Train a stepping/walking policy that stays upright on sand
- [ ] Train a stepping/walking policy that stays upright on water (or surface-running gait)
- [ ] Domain randomization across pool depth, particle density, friction
- [ ] Sim-to-real transfer notes

## License

This project is licensed under the Apache License 2.0 — see `LICENSE` for
details. The `assets/*_no_floor.xml` files are derivatives of the
[DeepMind Control Suite](https://github.com/google-deepmind/dm_control) models
bundled with Genesis, which are themselves Apache-2.0.

The Docker image installs upstream Genesis at build time from
<https://github.com/Genesis-Embodied-AI/Genesis>; see that repository for its
own license.
