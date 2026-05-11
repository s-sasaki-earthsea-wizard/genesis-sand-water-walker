"""Drive the planar walker to march in place on a flat rigid floor.

First step toward stepping on sand/water: no MPM medium, no drop. The walker
spawns just above a rigid plane, settles, then steps in place under an
adaptive controller that updates the per-step hip amplitude from torso state
so the gait shape does not need re-tuning when the user changes the cadence
(``--gait-hz``) or lift height (``--knee-amplitude``).

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
# GAIT_HZ (marching cadence) and KNEE_AMPLITUDE (foot lift height) are exposed
# via argparse so they can be swept without editing the file. The defaults
# below are the tuned baseline; the adaptive controller's integrator absorbs
# changes to GAIT_HZ, and tolerates moderate changes to KNEE_AMPLITUDE.
DEFAULT_GAIT_HZ = 1.0
DEFAULT_KNEE_AMPLITUDE = 0.6
SETTLE_TIME = 0.30       # s, hold the default pose before starting the gait


# ---------- Adaptive controller (full PID, mixed timescale) -----------------
# The hip_amp added to both hip targets each tick is the sum of three terms:
#
#   P (per-tick):  proportional to torso pitch + x  -> fast disturbance reject
#   I (per-step):  integrator updated only at half-cycle boundaries -> learns
#                  the steady-state lean needed to cancel the walker's
#                  structural backward-tip bias (feet attach +0.06 m ahead of
#                  the hip pivot in walker_no_floor.xml). Updating discretely
#                  rather than every tick avoids fighting the swing dynamics
#                  while still tracking slow drift.
#   D (per-tick):  proportional to pitch rate (+ x rate) -> damps oscillation
#
# Keeping the I term step-triggered is the "C" half of the design; the P+D
# pair is the "B" half. Together they let the user change GAIT_HZ or
# KNEE_AMPLITUDE without re-tuning the swing amplitude — the controller
# discovers it.
# Hip channel — pitch control.
P_PITCH = 2.0                   # rad / rad of pitch
D_PITCH = 0.3                   # rad / (rad/s) of pitch rate
I_PITCH = 0.6                   # I increment per step per rad of pitch
I_X = 0.4                       # I increment per step per m of x (residual)
HIP_BALANCE_LIMIT = 0.6         # rad, clamp on hip balance offset
I_LIMIT = 0.5                   # rad, clamp on the integrator (swing amplitude)

# Ankle channel — x position control. Ankle torque on the stance foot
# produces a near-horizontal force at the foot contact (~F = torque /
# foot_half_length) that translates the torso without strongly affecting
# pitch. This decouples x from pitch, which hip alone could not do.
P_X_ANKLE = 1.5                 # rad / m of x
D_X_ANKLE = 0.8                 # rad / (m/s) of x rate
ANKLE_LIMIT = 0.6               # rad, clamp on ankle target (XML range is ±0.785)


@dataclass
class GaitState:
    step_idx: int = -1
    hip_amp_i: float = 0.0       # integrator state (updated at step boundaries)


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

    Three superposed channels:
      * Swing (asymmetric, hip): ``hip_amp_i * *_lift`` — lifts the swing leg
        forward. Amplitude adapted by the per-step integrator.
      * Pitch balance (symmetric, hip): P+D on torso pitch — both hips share
        the same offset, applied every tick.
      * X balance (symmetric, ankle): P+D on rootx — both ankles share the
        same offset. Stance ankle's grounded torque -> horizontal foot force
        -> torso translation, with minimal pitch coupling. Swing ankle's
        contribution is in air and just rotates the swing foot harmlessly.
    """
    if t < SETTLE_TIME:
        return np.zeros(6)
    phase = 2.0 * np.pi * gait_hz * (t - SETTLE_TIME)
    s = np.sin(phase)
    r_lift = max(0.0, s)
    l_lift = max(0.0, -s)

    swing_r = state.hip_amp_i * r_lift
    swing_l = state.hip_amp_i * l_lift

    pitch_balance = -P_PITCH * pitch - D_PITCH * pitch_rate
    pitch_balance = float(np.clip(pitch_balance, -HIP_BALANCE_LIMIT, HIP_BALANCE_LIMIT))

    x_balance = -P_X_ANKLE * x - D_X_ANKLE * x_rate
    x_balance = float(np.clip(x_balance, -ANKLE_LIMIT, ANKLE_LIMIT))

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
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    print(
        f"[walker_marching] gait_hz={args.gait_hz} knee_amplitude={args.knee_amplitude}",
        flush=True,
    )

    os.makedirs(OUT_DIR, exist_ok=True)
    gs.init(precision="32", logging_level="info")

    dt = 4e-3
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=dt, substeps=10),
        show_viewer=False,
    )

    scene.add_entity(
        material=gs.materials.Rigid(),
        morph=gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True),
    )

    walker = scene.add_entity(
        material=gs.materials.Rigid(),
        morph=gs.morphs.MJCF(
            file=os.path.join(ASSETS_DIR, "walker_no_floor.xml"),
            pos=(0.0, 0.0, 0.05),
        ),
    )

    cam = scene.add_camera(
        res=(1280, 720),
        pos=(3.0, 2.6, 1.8),
        lookat=(0.0, 0.0, 1.0),
        fov=45,
        GUI=False,
    )

    scene.build(n_envs=0)

    dof_idx = [walker.get_joint(n).dofs_idx_local[0] for n in CONTROL_JOINTS]
    walker.set_dofs_kp(KP, dofs_idx_local=dof_idx)
    walker.set_dofs_kv(KV, dofs_idx_local=dof_idx)

    link_names = [l.name for l in walker.links]
    torso_idx = link_names.index("torso")

    out_video = os.path.join(OUT_DIR, "walker_marching.mp4")
    out_csv = os.path.join(OUT_DIR, "walker_marching.csv")
    cam.start_recording()

    horizon = 600 if "PYTEST_VERSION" not in os.environ else 5
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
                f"[walker_marching] {tag} step={i:4d} t={sim_t:.3f}s "
                f"torso=({torso[0]:+.3f},{torso[1]:+.3f},{torso[2]:+.3f}) "
                f"pitch={pitch:+.3f} hip_amp_i={state.hip_amp_i:+.3f}",
                flush=True,
            )

    final = log_rows[-1]
    print(
        f"[walker_marching] FINAL t={final[1]:.3f}s "
        f"torso_pos=({final[2]:+.4f},{final[3]:+.4f},{final[4]:+.4f}) "
        f"torso_vel=({final[5]:+.4f},{final[6]:+.4f},{final[7]:+.4f}) "
        f"hip_amp_i={state.hip_amp_i:+.4f}",
        flush=True,
    )

    cam.stop_recording(save_to_filename=out_video, fps=30)
    print(f"[walker_marching] wrote video {out_video}", flush=True)

    n_q = walker.n_qs
    with open(out_csv, "w") as f:
        header = (
            ["step", "t", "torso_x", "torso_y", "torso_z", "torso_vx", "torso_vy", "torso_vz"]
            + [f"q{j}" for j in range(n_q)]
        )
        f.write(",".join(header) + "\n")
        for row in log_rows:
            f.write(",".join(str(v) for v in row) + "\n")
    print(f"[walker_marching] wrote csv {out_csv}", flush=True)


if __name__ == "__main__":
    main()
