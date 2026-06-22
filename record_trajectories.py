#!/usr/bin/env python
"""
record_trajectories.py — render a stored demo to MP4 WITHOUT opening a GUI window.

Headless: connects in p.DIRECT and renders frames with getCameraImage, piping
them to ffmpeg. No window, far lighter than the interactive GUI, works where GUI
capture fails.

Renderer: uses the EGL hardware (GPU) renderer when available — it supports
TRANSPARENCY (translucent goal markers) and is fast. Falls back to the CPU
TinyRenderer if EGL can't load, in which case translucent goals appear opaque.

Drones: the real Crazyflie model (cf2x.urdf), shown in its natural detailed look.
Colour comes from per-drone trails and translucent goal markers. Trails are ON by
default (--no-trails to disable). --color flat-tints the drone bodies but loses
the mesh detail (off by default). --spheres forces simple spheres.

Usage:
    python record_trajectories.py <name> [-o OUT.mp4] [--downsample N] [--fps N]
        [--width W] [--height H] [--yaw deg] [--pitch deg] [--scale S]
        [--no-trails] [--color] [--spheres]

    python record_trajectories.py moving_goals_8
    python record_trajectories.py antipodal_swap_8 --downsample 2 --width 1920 --height 1080
"""
import argparse
import os
import pkgutil
import shutil
import subprocess
import sys

import numpy as np
import pybullet as p
import pybullet_data

_HERE = os.path.dirname(os.path.abspath(__file__))   # repo root
sys.path.insert(0, os.path.join(_HERE, 'viz'))
from viz_trajectory_goals import TrajectoryVisualizer  # noqa: E402

COLORS = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1]]


def _try_egl():
    """Load the EGL renderer plugin; return True if hardware rendering is up."""
    try:
        egl = pkgutil.get_loader('eglRenderer')
        pid = (p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
               if egl is not None else p.loadPlugin("eglRendererPlugin"))
        return pid is not None and pid >= 0
    except Exception:
        return False


def _add_tube(a, b, rgb, radius=0.012, alpha=0.5):
    """Add a thin translucent cylinder from a to b (a persistent trail segment)."""
    a = np.asarray(a, float); b = np.asarray(b, float)
    d = b - a; L = float(np.linalg.norm(d))
    if L < 1e-6:
        return
    dn = d / L; z = np.array([0, 0, 1.0])
    ax = np.cross(z, dn); s = float(np.linalg.norm(ax)); c = float(np.dot(z, dn))
    if s < 1e-9:
        orn = [0, 0, 0, 1] if c > 0 else [1, 0, 0, 0]
    else:
        ax = ax / s; half = np.arctan2(s, c) / 2.0
        orn = [*(ax * np.sin(half)), np.cos(half)]
    vs = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=L,
                             rgbaColor=rgb + [alpha])
    p.createMultiBody(0, baseVisualShapeIndex=vs,
                      basePosition=((a + b) / 2.0).tolist(), baseOrientation=orn)


def main():
    ap = argparse.ArgumentParser(description='Headless MP4 recorder for a stored demo')
    ap.add_argument('name', help='demo folder name under demos/')
    ap.add_argument('-o', '--out', default=None, help='output mp4 (default: demos/<name>/demo.mp4)')
    ap.add_argument('-d', '--downsample', type=int, default=5,
                    help='keep every Nth trajectory point as a frame; lower=more '
                         'frames/longer video (default: 5 -> 20 s at 30 fps)')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--yaw', type=float, default=45.0)
    ap.add_argument('--pitch', type=float, default=-35.0)
    ap.add_argument('--scale', type=float, default=1.0,
                    help='cf2x model scale; 1.0 = real Crazyflie size (default: 1.0)')
    ap.add_argument('--color', action='store_true',
                    help='flat-tint each drone (loses model detail; default: natural model)')
    ap.add_argument('--spheres', action='store_true',
                    help='force simple spheres instead of the cf2x drone model')
    ap.add_argument('--trails', action=argparse.BooleanOptionalAction, default=True,
                    help='draw each drone path as tube geometry (on by default; --no-trails to disable)')
    ap.add_argument('--trail-step', type=float, default=0.06,
                    help='min metres of travel between trail segments (default: 0.06)')
    ap.add_argument('--trail-width', type=float, default=0.012,
                    help='trail tube radius in metres (default: 0.012)')
    ap.add_argument('--trail-alpha', type=float, default=0.5,
                    help='trail opacity, 0=invisible..1=opaque (default: 0.5)')
    args = ap.parse_args()

    if shutil.which('ffmpeg') is None:
        sys.exit("ffmpeg not found in PATH — install it (sudo apt install ffmpeg).")

    base = os.path.join(_HERE, 'demos', args.name)
    traj = os.path.join(base, 'trajectories.txt')
    goals = os.path.join(base, 'goals.txt')
    out = os.path.abspath(args.out or os.path.join(base, 'demo.mp4'))
    if not os.path.exists(traj):
        sys.exit(f"Missing {traj} — run demos/generate_demos.sh first.")

    viz = TrajectoryVisualizer(traj, goals)
    pk = viz.pk[:, ::args.downsample, :]          # (N, T, 3)
    N, T = pk.shape[0], pk.shape[1]
    goal_tr = viz.goal_trajectories
    if goal_tr is not None:                        # resample goals to T frames
        idx = np.linspace(0, goal_tr.shape[1] - 1, T).astype(int)
        goal_tr = goal_tr[:, idx, :]

    p.connect(p.DIRECT)
    hw = _try_egl()
    renderer = p.ER_BULLET_HARDWARE_OPENGL if hw else p.ER_TINY_RENDERER
    if not hw:
        print("note: EGL hardware renderer unavailable — using TinyRenderer; "
              "translucent goals will appear opaque.")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    # Drone bodies: real Crazyflie model (cf2x.urdf) if available, else spheres.
    cf_urdf = None
    if not args.spheres:
        try:
            import pkg_resources
            cand = os.path.join(
                pkg_resources.resource_filename('gym_pybullet_drones', 'assets'),
                'cf2x.urdf')
            cf_urdf = cand if os.path.exists(cand) else None
        except Exception:
            cf_urdf = None
        if cf_urdf is None:
            print("note: cf2x.urdf not found (gym-pybullet-drones) — using spheres")

    drone_ids = []
    for i in range(N):
        color = COLORS[i % len(COLORS)] + [1]
        if cf_urdf:
            bid = p.loadURDF(cf_urdf, pk[i, 0, :].tolist(),
                             globalScaling=args.scale, useFixedBase=True)
            if args.color:                              # recolor every visual
                for j in range(-1, p.getNumJoints(bid)):
                    p.changeVisualShape(bid, j, rgbaColor=color)
        else:
            vs = p.createVisualShape(p.GEOM_SPHERE, radius=0.15, rgbaColor=color)
            bid = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs,
                                    basePosition=pk[i, 0, :].tolist())
        drone_ids.append(bid)
    goal_ids = []
    if goal_tr is not None:
        for i in range(goal_tr.shape[0]):
            vs = p.createVisualShape(p.GEOM_SPHERE, radius=0.2,
                                     rgbaColor=COLORS[i % len(COLORS)] + [0.35])
            goal_ids.append(p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs,
                                              basePosition=goal_tr[i, 0, :].tolist()))

    # Camera framed to the actual motion (drones + goals), not the workspace box
    # — the box can be far larger than the action (e.g. moving_goals_8), which
    # would shrink the drones to specks.
    pts = pk.reshape(-1, 3)
    if goal_tr is not None:
        pts = np.concatenate([pts, goal_tr.reshape(-1, 3)], axis=0)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    center = ((lo + hi) / 2.0).tolist()
    span = float(np.max(hi - lo))
    view = p.computeViewMatrixFromYawPitchRoll(center, span * 1.2 + 1.0, args.yaw,
                                               args.pitch, 0, 2)
    proj = p.computeProjectionMatrixFOV(60, args.width / args.height, 0.1, 1000)

    cmd = ['ffmpeg', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgba',
           '-s', f'{args.width}x{args.height}', '-r', str(args.fps), '-i', '-',
           '-an', '-vcodec', 'libx264', '-pix_fmt', 'yuv420p', out]
    ff = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    trail_anchor = [pk[i, 0, :].copy() for i in range(N)]

    for t in range(T):
        for i in range(N):
            p.resetBasePositionAndOrientation(drone_ids[i], pk[i, t, :].tolist(), [0, 0, 0, 1])
        if goal_tr is not None:
            for i in range(len(goal_ids)):
                p.resetBasePositionAndOrientation(goal_ids[i], goal_tr[i, t, :].tolist(), [0, 0, 0, 1])
        if args.trails:
            # Lay a tube only after the drone has travelled --trail-step metres,
            # so trails stay clean and never clump where a drone dwells (which
            # would otherwise bury the drone), independent of --downsample.
            for i in range(N):
                if np.linalg.norm(pk[i, t, :] - trail_anchor[i]) >= args.trail_step:
                    _add_tube(trail_anchor[i], pk[i, t, :], COLORS[i % len(COLORS)],
                              radius=args.trail_width, alpha=args.trail_alpha)
                    trail_anchor[i] = pk[i, t, :].copy()
        w, h, rgb, _, _ = p.getCameraImage(args.width, args.height, view, proj,
                                           renderer=renderer)
        frame = np.reshape(np.asarray(rgb, dtype=np.uint8), (h, w, 4))
        ff.stdin.write(frame.tobytes())
        if t % 25 == 0 or t == T - 1:
            print(f"\rrendering {t + 1}/{T} frames", end='', flush=True)

    print()
    ff.stdin.close()
    ff.wait()
    p.disconnect()
    print(f"wrote {out}")


if __name__ == '__main__':
    main()
