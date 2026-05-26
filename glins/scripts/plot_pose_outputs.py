#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def ecef_to_llh(xyz: np.ndarray) -> np.ndarray:
    x, y, z = xyz
    lon = math.atan2(y, x)
    lat = math.atan2(z, math.hypot(x, y))
    height = 0.0
    for _ in range(10):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = math.hypot(x, y) / max(math.cos(lat), 1e-12) - n
        new_lat = math.atan2(z, math.hypot(x, y) * (1.0 - WGS84_E2 * n / (n + height)))
        if abs(new_lat - lat) < 1e-13:
            lat = new_lat
            break
        lat = new_lat
    return np.array([lat, lon, height], dtype=float)


def ecef_to_enu(xyz: np.ndarray, ref_xyz: np.ndarray) -> np.ndarray:
    lat, lon, _ = ecef_to_llh(ref_xyz)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    rot = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=float,
    )
    return (rot @ (xyz - ref_xyz).T).T


def gps_to_unix(week: np.ndarray, sec: np.ndarray) -> np.ndarray:
    return week * 604800.0 + sec


def load_numeric_rows(path: Path) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line[0] in "%#":
                continue
            parts = line.split()
            try:
                rows.append([float(v) for v in parts])
            except ValueError:
                continue
    if not rows:
        raise ValueError(f"no numeric rows found in {path}")
    width = min(len(r) for r in rows)
    return np.array([r[:width] for r in rows], dtype=float)


def load_pose(path: Path) -> dict:
    data = load_numeric_rows(path)
    if data.shape[1] >= 8 and data[0, 0] > 1e9:
        return {
            "path": path,
            "kind": "local",
            "time": data[:, 0],
            "pos": data[:, 1:4],
            "q": None,
            "count": None,
        }
    if data.shape[1] >= 5:
        return {
            "path": path,
            "kind": "ecef",
            "time": gps_to_unix(data[:, 0], data[:, 1]),
            "pos": data[:, 2:5],
            "q": data[:, 5] if data.shape[1] > 5 else None,
            "count": data[:, 6] if data.shape[1] > 6 else None,
        }
    raise ValueError(f"unsupported pose format in {path}, got {data.shape[1]} columns")


def build_parser() -> argparse.ArgumentParser:
    root = Path(__file__).resolve().parents[4]
    default_files = [
        root / "output" / "gins.pos",
        # root / "output" / "rtklib.pos",
        # root / "output" / "rtklib_fgo.pos",
        # root / "output" / "gins_lio_increment.pos",
        # root / "output" / "total.pos",
    ]
    parser = argparse.ArgumentParser(description="Plot GLINS/GINS/RTK pose outputs.")
    parser.add_argument("files", nargs="*", type=Path, default=default_files)
    parser.add_argument("--labels", nargs="*", default=None)
    parser.add_argument("--out", type=Path, default=root / "output" / "pose_outputs.png")
    parser.add_argument("--ref", nargs=3, type=float, default=None, metavar=("X", "Y", "Z"))
    parser.add_argument("--show", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    existing = [p for p in args.files if p.exists() and p.stat().st_size > 0]
    if not existing:
        raise SystemExit("no input pose files found")

    poses = [load_pose(p) for p in existing]
    labels = args.labels or [p["path"].stem for p in poses]
    if len(labels) != len(poses):
        raise SystemExit("--labels length must match input file count")

    ecef_poses = [p for p in poses if p["kind"] == "ecef"]
    ref_xyz = np.array(args.ref, dtype=float) if args.ref else None
    if ref_xyz is None and ecef_poses:
        ref_xyz = ecef_poses[0]["pos"][0].copy()

    colors = ["tab:red", "tab:blue", "tab:green", "tab:purple", "tab:orange", "tab:brown"]

    fig = plt.figure(figsize=(16, 12))
    grid = fig.add_gridspec(4, 2, width_ratios=[1.2, 1.0], height_ratios=[1, 1, 1, 0.75], hspace=0.24, wspace=0.18)
    ax_xy = fig.add_subplot(grid[0:3, 0])
    ax_x = fig.add_subplot(grid[0, 1])
    ax_y = fig.add_subplot(grid[1, 1], sharex=ax_x)
    ax_z = fig.add_subplot(grid[2, 1], sharex=ax_x)
    ax_q = fig.add_subplot(grid[3, :])

    any_quality = False
    for idx, (pose, label) in enumerate(zip(poses, labels)):
        color = colors[idx % len(colors)]
        if pose["kind"] == "ecef":
            if ref_xyz is None:
                continue
            xyz = ecef_to_enu(pose["pos"], ref_xyz)
            coord_label = "ENU"
        else:
            xyz = pose["pos"] - pose["pos"][0]
            coord_label = "local relative"

        rel_t = pose["time"] - pose["time"][0]
        ax_xy.plot(xyz[:, 0], xyz[:, 1], linewidth=1.2, color=color, label=f"{label} ({coord_label})")
        ax_xy.scatter(xyz[0, 0], xyz[0, 1], color=color, s=28, marker="o")
        ax_xy.scatter(xyz[-1, 0], xyz[-1, 1], color=color, s=36, marker="x")
        ax_x.plot(rel_t, xyz[:, 0], linewidth=1.0, color=color, label=label)
        ax_y.plot(rel_t, xyz[:, 1], linewidth=1.0, color=color)
        ax_z.plot(rel_t, xyz[:, 2], linewidth=1.0, color=color)

        if pose["q"] is not None:
            any_quality = True
            ax_q.step(rel_t, pose["q"], where="post", linewidth=1.0, color=color, label=f"{label} Q")
        print(
            f"{label}: {len(xyz)} poses, format={pose['kind']}, "
            f"time {pose['time'][0]:.3f}->{pose['time'][-1]:.3f}"
        )

    ax_xy.set_title("Pose XY")
    ax_xy.set_xlabel("X / East (m)")
    ax_xy.set_ylabel("Y / North (m)")
    ax_xy.axis("equal")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best")

    for ax, name in zip([ax_x, ax_y, ax_z], ["X / East (m)", "Y / North (m)", "Z / Up (m)"]):
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
    ax_z.set_xlabel("Relative time (s)")
    ax_x.legend(loc="best")

    if any_quality:
        ax_q.set_title("Quality State")
        ax_q.set_ylabel("Q")
        ax_q.set_xlabel("Relative time (s)")
        ax_q.grid(True, alpha=0.25)
        ax_q.legend(loc="best")
    else:
        ax_q.axis("off")

    if ref_xyz is not None:
        fig.suptitle(f"Reference ECEF: {ref_xyz[0]:.3f}, {ref_xyz[1]:.3f}, {ref_xyz[2]:.3f}")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=180, bbox_inches="tight")
    print(f"saved: {args.out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
