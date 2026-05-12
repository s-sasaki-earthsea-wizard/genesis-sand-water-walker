# genesis-sand-water-walker

Soft-terrain locomotion experiments built on top of
[Genesis](https://github.com/Genesis-Embodied-AI/Genesis): we drop a humanoid
into a sand pool and a planar walker into a water pool, using Genesis's
MPM ↔ rigid-body coupling. A first stepping controller for the planar walker
on a rigid floor is also included as the precursor to walking on sand/water.

The long-term goal is **stepping/walking on sand and water** — a controller
that keeps a bipedal robot upright while interacting with a deformable medium.

日本語版は [readme/README.ja.md](readme/README.ja.md) を参照してください。

## Demos

Each script produces an MP4 plus a per-step CSV of the torso trajectory. Output
goes to `outputs/`.

| Scenario | Script | Output |
| --- | --- | --- |
| Humanoid → 0.75 m deep sand, 3.6 m drop | `scripts/humanoid_on_sand.py` | `outputs/humanoid_on_sand.{mp4,csv}` |
| Planar walker → 0.75 m deep water, 3.2 m drop | `scripts/walker_on_water.py` | `outputs/walker_on_water.{mp4,csv}` |
| Planar walker marches in place on rigid floor | `scripts/walker_marching.py` | `outputs/walker_marching.{mp4,csv}` |
| Planar walker marches in place in knee-deep water | `scripts/walker_marching_in_water.py` | `outputs/walker_marching_in_water.{mp4,csv}` |

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
make march-walker        # walker marches in place on rigid floor (~1 min)
make march-walker-in-water  # walker marches with lower legs in a water pool
```

The marching cadence and lift height can be overridden on the command line:

```bash
make march-walker GAIT_HZ=2.0                      # 2 Hz cadence
make march-walker KNEE_AMPLITUDE=0.9               # higher foot lift
make march-walker GAIT_HZ=2.0 KNEE_AMPLITUDE=0.9   # both
```

The in-water variant accepts the same knobs plus `WATER_LEVEL` (top of the
pool, in metres above the rigid floor; default 0.5 m ≈ knee height):

```bash
make march-walker-in-water WATER_LEVEL=0.4         # shallower pool (mid-shin)
make march-walker-in-water GAIT_HZ=1.5 WATER_LEVEL=0.5
make march-walker-in-water DURATION=5.0            # longer recording (default 2.4 s)
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
│   ├── walker_on_water.py
│   ├── walker_marching.py          # adaptive PID gait on rigid floor
│   └── walker_marching_in_water.py # same gait, lower legs submerged
├── assets/
│   ├── humanoid_no_floor.xml   # MJCF copies of Genesis's bundled models
│   └── walker_no_floor.xml     # with the worldbody floor plane removed
├── readme/
│   └── README.ja.md       # Japanese translation of this file
└── outputs/               # generated videos + CSVs (gitignored)
```

## Marching controller (walker_marching.py)

The walker model is planar (3 unactuated root DoFs: `rootz` slide, `rootx`
slide, `rooty` hinge) with 6 actuated joints — hip / knee / ankle on each leg.
The controller is a small PID with a deliberate role split:

| Channel | Actuator | Term(s) | Purpose |
| --- | --- | --- | --- |
| Swing | hip (asymmetric) | I, updated per step | Lift the swing leg forward; the integrator absorbs cadence changes |
| Pitch balance | hip (symmetric) | P + D | Keep `rooty` near zero |
| X balance | ankle (symmetric) | P + D | Hold `rootx` near zero via stance-foot horizontal force |

Routing pitch through hip torque and x through ankle torque is what lets the
controller hold both states simultaneously — the walker's foot attaches +0.06 m
in front of the hip pivot, so hip-only control couples the two and ends up
either drifting forward or tipping backward.

At the baseline (`GAIT_HZ=1.0`, `KNEE_AMPLITUDE=0.6`), after a 0.3 s settle
and 2.1 s of marching the walker holds `|x| ≤ 2 cm` and `|pitch| ≤ 1°`.
Doubling the cadence is absorbed by the integrator without re-tuning. Raising
the knee amplitude beyond ~0.8 rad introduces forward drift (the P/D gains on
the ankle channel would need to be scaled with lift) — a documented limitation
to address before moving to sand/water.

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
- [x] Walker marches in place on rigid floor (adaptive PID, `make march-walker`)
- [x] Walker marches in place in knee-deep water (integrated scene, `make march-walker-in-water`) — `|x| ≤ 2.2 cm`, `|pitch| ≤ 1.1°` at baseline, on par with the dry-floor controller
- [ ] Walker marches in place on sand
- [ ] Step forward / stepping policy on rigid, then transferred to sand/water
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
