"""Walker marches in place with its lower legs in a knee-deep water pool.

Combines the rigid-floor adaptive-PID gait controller from
``walker_marching.py`` with the MPM water pool from ``walker_on_water.py``.
The walker spawns just above a rigid plane that doubles as the pool bottom,
with the water surface set to roughly knee height (default 0.5 m, the knee
pivot sits near 0.55 m above the floor).

Saves an MP4 and a per-step CSV of the torso trajectory and full qpos to
``outputs/``.
"""
import argparse
import os
from dataclasses import dataclass

import numpy as np
import torch

import genesis as gs


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
ASSETS_DIR = os.path.join(PROJECT_ROOT, "assets")
OUT_DIR = os.path.join(PROJECT_ROOT, "outputs")


# Actuated joints (the 3 root DoFs are unactuated by design — walker is planar).
CONTROL_JOINTS = [
    "right_hip", "left_hip",
    "right_knee", "left_knee",
    "right_ankle", "left_ankle",
]

# Joint-level PD gains for the rigid solver. Stance-side knee kp must be high
# enough to hold body weight under the swing-side lift.
KP = np.array([200.0, 200.0, 80.0, 80.0, 40.0, 40.0])
KV = np.array([10.0, 10.0, 6.0, 6.0, 3.0, 3.0])


# ---------- User knobs ------------------------------------------------------
DEFAULT_GAIT_HZ = 1.0
DEFAULT_KNEE_AMPLITUDE = 0.6
DEFAULT_WATER_LEVEL = 0.5        # m, top of the water pool ~ knee height
DEFAULT_DURATION = 2.4           # s of simulated time recorded in the main loop
# Time before the gait (swing + knee) is allowed to start. Unlike the dry
# walker_marching script, the balance channels (pitch_balance on hips,
# x_balance on ankles) are active *from t=0* in this script — water exerts
# a chronic disturbance on the lower legs during the fall and after
# touchdown, and gating the PD off during settle let drift accumulate to a
# point the controller could not recover from.
SETTLE_TIME = 1.0

# Steps to pre-step the scene before the recorded main loop begins.
# An MPM Box is sampled with particles at rest and zero internal pressure,
# so the column briefly free-falls and oscillates until hydrostatic pressure
# builds (~0.05 s). If the walker is already in contact with the water at
# t=0, that transient kicks the lower legs and biases the walker's pose
# before the controller is even active. Pre-settling the water with the
# walker held a few cm above the surface decouples the two transients.
PRESETTLE_STEPS = 250        # ~1.0 s of pre-stepping at dt=4 ms


# ---------- Adaptive controller (full PID, mixed timescale) -----------------
# Pitch channel inherits the dry-floor gains (it works well — the walker
# stays within a few degrees in water with the balance-during-settle fix).
# The x channel needs more authority than on dry floor: each swing cycle
# drags the lifted leg through water, which by Newton's third law pushes
# the body backward, and the dry-floor ANKLE gains can't recover the lost
# ground before the next swing adds more drag. Doubling P/D and opening
# the ankle clamp to the full MJCF range (±0.785 rad) gives the stance
# foot enough horizontal-force budget per cycle.
P_PITCH = 2.0
D_PITCH = 0.3
I_PITCH = 0.6
I_X = 0.4
HIP_BALANCE_LIMIT = 0.6
I_LIMIT = 0.5

P_X_ANKLE = 3.0
D_X_ANKLE = 1.5
ANKLE_LIMIT = 0.785


@dataclass
class GaitState:
    step_idx: int = -1
    hip_amp_i: float = 0.0


def _np(x):
    return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)


def update_step_adaptation(
    state: GaitState, t: float, pitch: float, x: float, gait_hz: float
) -> bool:
    """Advance the per-step integrator at half-cycle boundaries.

    Returns True iff this tick crossed a step boundary (useful for logging).
    """
    if t < SETTLE_TIME:
        return False
    step_idx = int((t - SETTLE_TIME) * 2.0 * gait_hz)
    if step_idx == state.step_idx:
        return False
    state.step_idx = step_idx
    delta = -I_PITCH * pitch - I_X * x
    state.hip_amp_i = float(np.clip(state.hip_amp_i + delta, -I_LIMIT, I_LIMIT))
    return True


def gait_targets(
    state: GaitState,
    t: float,
    pitch: float,
    pitch_rate: float,
    x: float,
    x_rate: float,
    gait_hz: float,
    knee_amplitude: float,
) -> np.ndarray:
    """PD position targets for [r_hip, l_hip, r_knee, l_knee, r_ankle, l_ankle].

    Balance terms (pitch on hips, x on ankles) are always active. The swing
    and knee lift terms only engage once t >= SETTLE_TIME.
    """
    pitch_balance = -P_PITCH * pitch - D_PITCH * pitch_rate
    pitch_balance = float(np.clip(pitch_balance, -HIP_BALANCE_LIMIT, HIP_BALANCE_LIMIT))

    x_balance = -P_X_ANKLE * x - D_X_ANKLE * x_rate
    x_balance = float(np.clip(x_balance, -ANKLE_LIMIT, ANKLE_LIMIT))

    if t < SETTLE_TIME:
        return np.array([pitch_balance, pitch_balance, 0.0, 0.0, x_balance, x_balance])

    phase = 2.0 * np.pi * gait_hz * (t - SETTLE_TIME)
    s = np.sin(phase)
    r_lift = max(0.0, s)
    l_lift = max(0.0, -s)

    swing_r = state.hip_amp_i * r_lift
    swing_l = state.hip_amp_i * l_lift

    r_hip = swing_r + pitch_balance
    l_hip = swing_l + pitch_balance
    r_knee = -knee_amplitude * r_lift
    l_knee = -knee_amplitude * l_lift
    r_ankle = x_balance
    l_ankle = x_balance
    return np.array([r_hip, l_hip, r_knee, l_knee, r_ankle, l_ankle])


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--gait-hz",
        type=float,
        default=DEFAULT_GAIT_HZ,
        help=f"Marching cadence in Hz (default: {DEFAULT_GAIT_HZ}).",
    )
    p.add_argument(
        "--knee-amplitude",
        type=float,
        default=DEFAULT_KNEE_AMPLITUDE,
        help=f"Swing-side knee bend in rad (default: {DEFAULT_KNEE_AMPLITUDE}).",
    )
    p.add_argument(
        "--water-level",
        type=float,
        default=DEFAULT_WATER_LEVEL,
        help=(
            "Top of the water pool in m, measured from the rigid floor at z=0 "
            f"(default: {DEFAULT_WATER_LEVEL}, ~knee height)."
        ),
    )
    p.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION,
        help=(
            "Length of the recorded simulation in seconds — the main loop "
            f"runs ceil(duration / dt) physics steps (default: {DEFAULT_DURATION})."
        ),
    )
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    print(
        f"[walker_marching_in_water] gait_hz={args.gait_hz} "
        f"knee_amplitude={args.knee_amplitude} water_level={args.water_level} "
        f"duration={args.duration}",
        flush=True,
    )

    os.makedirs(OUT_DIR, exist_ok=True)
    gs.init(precision="32", logging_level="info")

    dt = 4e-3
    # Pool sits on the rigid floor (z=0). Depth equals --water-level so the
    # surface lands at z = water_level.
    pool_xy = 0.8
    pool_depth = args.water_level
    pool_center_z = 0.5 * pool_depth

    # MPM safety padding (~3 / grid_density ≈ 0.075 m on each axis) shrinks the
    # usable region inside lower/upper_bound, so we drop the z floor to -0.10
    # to leave room for the water box to start at z=0 (above the rigid plane).
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=dt, substeps=25),
        mpm_options=gs.options.MPMOptions(
            lower_bound=(-0.6, -0.6, -0.10),
            upper_bound=(0.6, 0.6, 1.0),
            grid_density=40,
        ),
        vis_options=gs.options.VisOptions(visualize_mpm_boundary=True),
        show_viewer=False,
    )

    scene.add_entity(
        material=gs.materials.Rigid(needs_coup=True, coup_friction=0.2),
        morph=gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True),
    )

    scene.add_entity(
        material=gs.materials.MPM.Liquid(),
        morph=gs.morphs.Box(
            pos=(0.0, 0.0, pool_center_z),
            size=(pool_xy, pool_xy, pool_depth),
        ),
        surface=gs.surfaces.Rough(color=(0.3, 0.55, 0.95, 0.9), vis_mode="particle"),
    )

    # Spawn the walker high enough above the water surface that pre-settling
    # (~0.1 s of free fall, ~5 cm drop) still leaves the feet above the water
    # when the main loop begins. The morph pos's z equals the foot-bottom
    # world z in the body's default pose, so feet-above-water reduces to
    # spawn_z > water_level. We want the lower legs not to overlap MPM
    # particles during pre-settle either, so the gap is set to ~10 cm.
    spawn_z = args.water_level + 0.10
    walker = scene.add_entity(
        material=gs.materials.Rigid(needs_coup=True, coup_friction=0.3),
        morph=gs.morphs.MJCF(
            file=os.path.join(ASSETS_DIR, "walker_no_floor.xml"),
            pos=(0.0, 0.0, spawn_z),
        ),
    )

    cam = scene.add_camera(
        res=(1280, 720),
        pos=(3.0, 2.6, 1.8),
        lookat=(0.0, 0.0, 0.8),
        fov=45,
        GUI=False,
    )

    scene.build(n_envs=0)

    dof_idx = [walker.get_joint(n).dofs_idx_local[0] for n in CONTROL_JOINTS]
    walker.set_dofs_kp(KP, dofs_idx_local=dof_idx)
    walker.set_dofs_kv(KV, dofs_idx_local=dof_idx)

    link_names = [l.name for l in walker.links]
    torso_idx = link_names.index("torso")

    # Pre-settle the water column with the walker held in its default pose
    # (PD targets zero, walker is above the water surface so there's no
    # coupling yet). After this loop, MPM particles are at hydrostatic
    # equilibrium and the main loop starts from a still water surface.
    presettle_target = np.zeros(6)
    for _ in range(PRESETTLE_STEPS):
        walker.control_dofs_position(presettle_target, dofs_idx_local=dof_idx)
        scene.step()
    print(
        f"[walker_marching_in_water] pre-settled water for {PRESETTLE_STEPS} steps "
        f"(~{PRESETTLE_STEPS * dt:.2f} s)",
        flush=True,
    )

    out_video = os.path.join(OUT_DIR, "walker_marching_in_water.mp4")
    out_csv = os.path.join(OUT_DIR, "walker_marching_in_water.csv")
    cam.start_recording()

    horizon = int(np.ceil(args.duration / dt)) if "PYTEST_VERSION" not in os.environ else 5
    log_rows = []
    state = GaitState()

    for i in range(horizon):
        sim_t = (i + 1) * dt

        qpos = _np(walker.get_qpos())
        qvel = _np(walker.get_dofs_velocity())
        x = float(qpos[1])
        pitch = float(qpos[2])
        x_rate = float(qvel[1])
        pitch_rate = float(qvel[2])

        stepped = update_step_adaptation(state, sim_t, pitch, x, args.gait_hz)
        target = gait_targets(
            state, sim_t, pitch, pitch_rate, x, x_rate,
            args.gait_hz, args.knee_amplitude,
        )

        walker.control_dofs_position(target, dofs_idx_local=dof_idx)
        scene.step()
        cam.render()

        torso = _np(walker.get_links_pos())[torso_idx]
        torso_vel = _np(walker.get_links_vel())[torso_idx]
        qpos = _np(walker.get_qpos())
        log_rows.append((i, sim_t, *torso, *torso_vel, *qpos.tolist()))

        if stepped or i % 30 == 0 or i == horizon - 1:
            tag = "STEP" if stepped else "    "
            print(
                f"[walker_marching_in_water] {tag} step={i:4d} t={sim_t:.3f}s "
                f"torso=({torso[0]:+.3f},{torso[1]:+.3f},{torso[2]:+.3f}) "
                f"pitch={pitch:+.3f} hip_amp_i={state.hip_amp_i:+.3f}",
                flush=True,
            )

    final = log_rows[-1]
    print(
        f"[walker_marching_in_water] FINAL t={final[1]:.3f}s "
        f"torso_pos=({final[2]:+.4f},{final[3]:+.4f},{final[4]:+.4f}) "
        f"torso_vel=({final[5]:+.4f},{final[6]:+.4f},{final[7]:+.4f}) "
        f"hip_amp_i={state.hip_amp_i:+.4f}",
        flush=True,
    )

    cam.stop_recording(save_to_filename=out_video, fps=30)
    print(f"[walker_marching_in_water] wrote video {out_video}", flush=True)

    n_q = walker.n_qs
    with open(out_csv, "w") as f:
        header = (
            ["step", "t", "torso_x", "torso_y", "torso_z", "torso_vx", "torso_vy", "torso_vz"]
            + [f"q{j}" for j in range(n_q)]
        )
        f.write(",".join(header) + "\n")
        for row in log_rows:
            f.write(",".join(str(v) for v in row) + "\n")
    print(f"[walker_marching_in_water] wrote csv {out_csv}", flush=True)


if __name__ == "__main__":
    main()
