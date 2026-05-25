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
    r2 = x * x + y * y
    lon = math.atan2(y, x)
    lat = math.atan2(z, math.sqrt(r2))

    height = 0.0
    for _ in range(10):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = math.sqrt(r2) / max(math.cos(lat), 1e-12) - n
        lat_next = math.atan2(z, math.sqrt(r2) * (1.0 - WGS84_E2 * n / (n + height)))
        if abs(lat_next - lat) < 1e-13:
            lat = lat_next
            break
        lat = lat_next

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


def load_pos(path: Path) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("%") or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 6:
                continue
            try:
                rows.append(
                    [
                        float(parts[0]),
                        float(parts[1]),
                        float(parts[2]),
                        float(parts[3]),
                        float(parts[4]),
                        int(float(parts[5])),
                        int(float(parts[6])) if len(parts) > 6 else 0,
                        float(parts[14]) if len(parts) > 14 else np.nan,
                    ]
                )
            except ValueError:
                continue

    if not rows:
        raise ValueError(f"no pose rows found in {path}")
    return np.array(rows, dtype=float)


def plot_dataset(ax_xy, ax_enu, ax_q, data: np.ndarray, ref_xyz: np.ndarray, label: str, color: str) -> None:
    time_s = (data[:, 0] - data[0, 0]) * 604800.0 + (data[:, 1] - data[0, 1])
    xyz = data[:, 2:5]
    enu = ecef_to_enu(xyz, ref_xyz)
    q = data[:, 5]
    ns = data[:, 6]

    ax_xy.plot(enu[:, 0], enu[:, 1], linewidth=1.4, color=color, label=label)
    ax_xy.scatter(enu[0, 0], enu[0, 1], s=35, color=color, marker="o")
    ax_xy.scatter(enu[-1, 0], enu[-1, 1], s=45, color=color, marker="x")

    ax_enu[0].plot(time_s, enu[:, 0], linewidth=1.1, color=color, label=label)
    ax_enu[1].plot(time_s, enu[:, 1], linewidth=1.1, color=color)
    ax_enu[2].plot(time_s, enu[:, 2], linewidth=1.1, color=color)

    ax_q.step(time_s, q, where="post", linewidth=1.0, color=color, label=f"{label} Q")
    ax_q_twin = ax_q.twinx() if not hasattr(ax_q, "_ns_axis") else ax_q._ns_axis
    ax_q._ns_axis = ax_q_twin
    ax_q_twin.plot(time_s, ns, linewidth=0.8, linestyle="--", color=color, alpha=0.55, label=f"{label} ns")

    print(
        f"{label}: {len(data)} poses, "
        f"GPST {int(data[0,0])} {data[0,1]:.3f} -> {int(data[-1,0])} {data[-1,1]:.3f}, "
        f"Q counts {dict(zip(*np.unique(q.astype(int), return_counts=True)))}"
    )


def build_parser() -> argparse.ArgumentParser:
    root = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(description="Plot RTK/FGO ECEF .pos outputs as local ENU pose graphs.")
    parser.add_argument(
        "files",
        nargs="*",
        type=Path,
        default=[root / "output" / "rtklib.pos", root / "output" / "rtklib_fgo.pos"],
        help="RTKLIB-style .pos files. Default: output/rtklib.pos output/rtklib_fgo.pos",
    )
    parser.add_argument("--labels", nargs="*", default=None, help="optional labels matching files")
    parser.add_argument("--out", type=Path, default=root / "output" / "rtk_pose_plot.png")
    parser.add_argument("--ref", nargs=3, type=float, default=None, metavar=("X", "Y", "Z"))
    parser.add_argument("--show", action="store_true", help="show interactive window after saving")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    files = [p for p in args.files if p.exists() and p.stat().st_size > 0]
    if not files:
        raise SystemExit("no input .pos files found")

    loaded = [(path, load_pos(path)) for path in files]
    ref_xyz = np.array(args.ref, dtype=float) if args.ref else loaded[0][1][0, 2:5]
    labels = args.labels or [path.stem for path, _ in loaded]
    if len(labels) != len(loaded):
        raise SystemExit("--labels length must match number of input files")

    fig = plt.figure(figsize=(15, 12))
    grid = fig.add_gridspec(
        4,
        2,
        width_ratios=[1.15, 1.0],
        height_ratios=[1.0, 1.0, 1.0, 0.75],
        hspace=0.24,
        wspace=0.18,
    )
    ax_xy = fig.add_subplot(grid[0:3, 0])
    ax_e = fig.add_subplot(grid[0, 1])
    ax_n = fig.add_subplot(grid[1, 1], sharex=ax_e)
    ax_u = fig.add_subplot(grid[2, 1], sharex=ax_e)
    ax_q = fig.add_subplot(grid[3, :])

    colors = ["tab:blue", "tab:red", "tab:green", "tab:purple", "tab:orange"]
    for idx, ((path, data), label) in enumerate(zip(loaded, labels)):
        plot_dataset(ax_xy, [ax_e, ax_n, ax_u], ax_q, data, ref_xyz, label, colors[idx % len(colors)])

    ax_xy.set_title("RTK Pose Graph (ENU)")
    ax_xy.set_xlabel("East (m)")
    ax_xy.set_ylabel("North (m)")
    ax_xy.axis("equal")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best")

    for ax, name in zip([ax_e, ax_n, ax_u], ["East (m)", "North (m)", "Up (m)"]):
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
    ax_u.set_xlabel("Relative time (s)")
    ax_e.legend(loc="best")

    ax_q.set_title("Quality")
    ax_q.set_xlabel("Relative time (s)")
    ax_q.set_ylabel("Q")
    ax_q.grid(True, alpha=0.25)
    ax_q.legend(loc="upper left", fontsize=8)
    if hasattr(ax_q, "_ns_axis"):
        ax_q._ns_axis.set_ylabel("ns")

    fig.suptitle(f"Reference ECEF: {ref_xyz[0]:.3f}, {ref_xyz[1]:.3f}, {ref_xyz[2]:.3f}")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=180, bbox_inches="tight")
    print(f"saved: {args.out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
