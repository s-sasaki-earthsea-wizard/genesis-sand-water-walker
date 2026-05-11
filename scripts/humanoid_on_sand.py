"""Drop a humanoid (MJCF) into a deep sand pool and record the dive.

Saves an MP4 and a per-step CSV of the torso trajectory to ``outputs/``.
"""
import os

import numpy as np
import torch

import genesis as gs


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
ASSETS_DIR = os.path.join(PROJECT_ROOT, "assets")
OUT_DIR = os.path.join(PROJECT_ROOT, "outputs")


def _np(x):
    return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    gs.init(precision="32", logging_level="info")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=4e-3, substeps=25),
        mpm_options=gs.options.MPMOptions(
            lower_bound=(-0.6, -0.6, -0.05),
            upper_bound=(0.6, 0.6, 1.3),
            grid_density=40,
        ),
        vis_options=gs.options.VisOptions(visualize_mpm_boundary=True),
        show_viewer=False,
    )

    scene.add_entity(
        material=gs.materials.Rigid(needs_coup=True, coup_friction=0.6),
        morph=gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True),
    )

    scene.add_entity(
        material=gs.materials.MPM.Sand(),
        morph=gs.morphs.Box(pos=(0.0, 0.0, 0.40), size=(0.8, 0.8, 0.75)),
        surface=gs.surfaces.Rough(color=(0.95, 0.82, 0.55, 1.0), vis_mode="particle"),
    )

    humanoid = scene.add_entity(
        material=gs.materials.Rigid(needs_coup=True, coup_friction=0.6),
        morph=gs.morphs.MJCF(
            file=os.path.join(ASSETS_DIR, "humanoid_no_floor.xml"),
            pos=(0.0, 0.0, 2.30),
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

    link_names = [l.name for l in humanoid.links]
    torso_idx = link_names.index("torso")
    has_links_vel = hasattr(humanoid, "get_links_vel")

    out_video = os.path.join(OUT_DIR, "humanoid_on_sand.mp4")
    out_csv = os.path.join(OUT_DIR, "humanoid_on_sand.csv")
    cam.start_recording()

    horizon = 600 if "PYTEST_VERSION" not in os.environ else 5
    log_rows = []

    for i in range(horizon):
        scene.step()
        cam.render()

        torso = _np(humanoid.get_links_pos())[torso_idx]
        torso_vel = _np(humanoid.get_links_vel())[torso_idx] if has_links_vel else np.zeros(3)
        sim_t = (i + 1) * 4e-3
        log_rows.append((i, sim_t, *torso, *torso_vel))

        if i % 30 == 0 or i == horizon - 1:
            print(
                f"[humanoid_on_sand] step={i:4d} t={sim_t:.3f}s "
                f"torso=({torso[0]:+.3f},{torso[1]:+.3f},{torso[2]:+.3f}) "
                f"v=({torso_vel[0]:+.3f},{torso_vel[1]:+.3f},{torso_vel[2]:+.3f})",
                flush=True,
            )

    final = log_rows[-1]
    print(
        f"[humanoid_on_sand] FINAL t={final[1]:.3f}s "
        f"torso_pos=({final[2]:+.4f},{final[3]:+.4f},{final[4]:+.4f}) "
        f"torso_vel=({final[5]:+.4f},{final[6]:+.4f},{final[7]:+.4f})",
        flush=True,
    )

    cam.stop_recording(save_to_filename=out_video, fps=30)
    print(f"[humanoid_on_sand] wrote video {out_video}", flush=True)

    with open(out_csv, "w") as f:
        f.write("step,t,torso_x,torso_y,torso_z,torso_vx,torso_vy,torso_vz\n")
        for row in log_rows:
            f.write(",".join(str(v) for v in row) + "\n")
    print(f"[humanoid_on_sand] wrote csv {out_csv}", flush=True)


if __name__ == "__main__":
    main()
