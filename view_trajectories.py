#!/usr/bin/env python
"""
view_trajectories.py — replay a stored demo in PyBullet so you can record it.

Reuses the existing TrajectoryVisualizer (no changes to viz/viz_trajectory_goals.py).

Usage:
    python view_trajectories.py <demo_name> [--downsample N] [--no-urdf] [--record OUT.mp4]
    python view_trajectories.py moving_goals_8
    python view_trajectories.py reallocation_predictive_4 --downsample 15
    python view_trajectories.py moving_goals_8 --record demos/moving_goals_8/demo.mp4

Demo names match the folders created by generate_demos.sh:
    antipodal_swap_8, moving_goals_8, reallocation_static_4,
    reallocation_predictive_4, scalability_32

Recording: --record writes an MP4 via PyBullet's built-in logger (needs ffmpeg
in PATH). Capture starts when the scene builds and stops when you close the
window (Ctrl+C in the terminal) — press Ctrl+C shortly after the replay finishes
to keep the clip tight. The GUI side panels are hidden while recording.
"""
import argparse
import os
import shutil
import sys

import pybullet as p

_HERE = os.path.dirname(os.path.abspath(__file__))   # repo root
sys.path.insert(0, os.path.join(_HERE, 'viz'))
from viz_trajectory_goals import TrajectoryVisualizer  # noqa: E402


def _enable_recording(record_path):
    """Patch pybullet.connect so MP4 logging starts right after the visualizer
    connects to the GUI (the visualizer owns the connect call, so we hook here)."""
    real_connect = p.connect

    def connect(*args, **kwargs):
        cid = real_connect(*args, **kwargs)
        # Hide the parameter/preview panels for a clean capture.
        try:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=cid)
        except Exception:
            pass
        p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, record_path,
                            physicsClientId=cid)
        return cid

    p.connect = connect


def main():
    parser = argparse.ArgumentParser(description='Replay a stored SwarmSync demo')
    parser.add_argument('name', help='demo folder name under demos/')
    parser.add_argument('--downsample', '-d', type=int, default=20,
                        help='trajectory downsampling factor (default: 20)')
    parser.add_argument('--no-urdf', action='store_true',
                        help='use spheres instead of the Crazyflie URDF')
    parser.add_argument('--record', '-r', metavar='OUT.mp4', default=None,
                        help='record an MP4 via PyBullet (requires ffmpeg in PATH)')
    args = parser.parse_args()

    base = os.path.join(_HERE, 'demos', args.name)
    traj = os.path.join(base, 'trajectories.txt')
    goals = os.path.join(base, 'goals.txt')

    if not os.path.isdir(base):
        sys.exit(f"No demo folder: {base}\nRun demos/generate_demos.sh first.")
    if not os.path.exists(traj):
        sys.exit(f"Missing {traj} — (re)run demos/generate_demos.sh.")

    print(f"Replaying demo '{args.name}'")
    print(f"  trajectory: {traj}")
    print(f"  goals:      {goals}")

    if args.record:
        rec = os.path.abspath(args.record)
        os.makedirs(os.path.dirname(rec) or '.', exist_ok=True)
        if shutil.which('ffmpeg') is None:
            print("  WARNING: ffmpeg not found in PATH — MP4 logging may produce "
                  "no file. Install ffmpeg, or use a screen recorder instead.")
        _enable_recording(rec)
        print(f"  recording:  {rec}")
        print("  -> press Ctrl+C shortly after replay finishes to stop & save.")

    viz = TrajectoryVisualizer(traj, goals)
    viz.visualize_in_pybullet(gui=True, downsample=args.downsample,
                              use_urdf=not args.no_urdf)


if __name__ == '__main__':
    main()
