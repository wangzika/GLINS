#!/usr/bin/env python3
"""
Visualize GLINS outputs locally without RViz.

Default inputs:
  - docker/output/total.pos
  - docker/output/MAP/globalmap_lidar_feature.pcd

The script intentionally uses only numpy + matplotlib so it is easy to run on
macOS host machines that cannot run RViz reliably through Docker/XQuartz.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np


def load_total_pos(path: Path) -> Optional[np.ndarray]:
    if not path.exists():
        print(f"[warn] trajectory file not found: {path}")
        return None
    if path.stat().st_size == 0:
        print(f"[warn] trajectory file is empty: {path}")
        return None

    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[np.newaxis, :]
    if data.shape[1] < 8:
        raise ValueError(
            f"unexpected trajectory format in {path}, expected >= 8 columns, got {data.shape[1]}"
        )
    return data


def load_ascii_pcd_xyz(path: Path) -> Optional[np.ndarray]:
    if not path.exists():
        print(f"[warn] point cloud file not found: {path}")
        return None
    if path.stat().st_size == 0:
        print(f"[warn] point cloud file is empty: {path}")
        return None

    fields = None
    data_type = None

    with path.open("r", encoding="utf-8", errors="ignore") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"invalid PCD header: {path}")
            line = line.strip()
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("DATA"):
                data_type = line.split()[1].lower()
                break

        if data_type != "ascii":
            raise ValueError(f"only ASCII PCD is supported, got DATA {data_type} in {path}")

        if not fields:
            raise ValueError(f"PCD header missing FIELDS in {path}")

        try:
            usecols = [fields.index("x"), fields.index("y"), fields.index("z")]
        except ValueError as exc:
            raise ValueError(f"PCD does not contain x/y/z fields: {path}") from exc

        points = np.loadtxt(f, usecols=usecols)

    if points.ndim == 1:
        points = points[np.newaxis, :]
    return points


def maybe_sample(points: np.ndarray, max_points: int, seed: int) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(points), size=max_points, replace=False)
    return points[idx]


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parents[2]
    default_traj = repo_root / "docker" / "output" / "total.pos"
    default_map = repo_root / "docker" / "output" / "MAP" / "globalmap_lidar_feature.pcd"

    parser = argparse.ArgumentParser(
        description="Visualize GLINS trajectory and map locally on macOS/Linux without RViz."
    )
    parser.add_argument("--traj", type=Path, default=default_traj, help="trajectory file (.pos)")
    parser.add_argument("--map", dest="map_path", type=Path, default=default_map, help="map file (.pcd)")
    parser.add_argument(
        "--max-points",
        type=int,
        default=200000,
        help="maximum number of point cloud samples to draw",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="random seed used when downsampling the point cloud",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="optional image output path; if omitted, an interactive window is shown",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()

    traj = load_total_pos(args.traj)
    points = load_ascii_pcd_xyz(args.map_path)

    if traj is None and points is None:
        raise SystemExit("no valid input found; nothing to visualize")

    if points is not None:
        points = maybe_sample(points, args.max_points, args.seed)

    fig, (ax_xy, ax_tz) = plt.subplots(1, 2, figsize=(15, 7))

    if points is not None:
        scatter = ax_xy.scatter(
            points[:, 0],
            points[:, 1],
            c=points[:, 2],
            s=0.3,
            alpha=0.35,
            cmap="viridis",
            linewidths=0,
        )
        cbar = fig.colorbar(scatter, ax=ax_xy, fraction=0.046, pad=0.04)
        cbar.set_label("Map Z (m)")

    if traj is not None:
        ax_xy.plot(traj[:, 1], traj[:, 2], color="crimson", linewidth=1.5, label="Trajectory")
        ax_xy.scatter(traj[0, 1], traj[0, 2], color="limegreen", s=40, label="Start", zorder=3)
        ax_xy.scatter(traj[-1, 1], traj[-1, 2], color="black", s=40, label="End", zorder=3)
        rel_time = traj[:, 0] - traj[0, 0]
        ax_tz.plot(rel_time, traj[:, 1], label="X", linewidth=1.2)
        ax_tz.plot(rel_time, traj[:, 2], label="Y", linewidth=1.2)
        ax_tz.plot(rel_time, traj[:, 3], label="Z", linewidth=1.2)
        ax_tz.legend()
    else:
        ax_tz.text(0.5, 0.5, "No trajectory loaded", ha="center", va="center", transform=ax_tz.transAxes)

    ax_xy.set_title("Map XY + Trajectory")
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.grid(True, alpha=0.25)
    if traj is not None:
        ax_xy.legend(loc="best")

    ax_tz.set_title("Trajectory vs Time")
    ax_tz.set_xlabel("Relative Time (s)")
    ax_tz.set_ylabel("Position (m)")
    ax_tz.grid(True, alpha=0.25)

    fig.suptitle("GLINS Local Visualization", fontsize=14)
    fig.tight_layout()

    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=180, bbox_inches="tight")
        print(f"[info] saved figure to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
