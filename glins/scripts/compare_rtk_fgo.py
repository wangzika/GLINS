#!/usr/bin/env python3
"""Compare RTKLIB RTK and GLINS FGO position outputs.

The script reads RTKLIB/GLINS `.pos` files with columns:

    week sow x_ecef y_ecef z_ecef ...

It converts both trajectories to ENU with respect to a reference ECEF point,
plots trajectory and component time series, and reports the FGO-RTK difference
at matched epochs.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import rtkcmn  # noqa: E402


def load_pos(path: Path) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line[0] in ("%", "#"):
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                row = [float(v) for v in parts[:5]]
            except ValueError:
                continue
            rows.append(row)
    if not rows:
        raise ValueError(f"no position rows found in {path}")
    return np.asarray(rows, dtype=float)


def to_seconds(data: np.ndarray) -> np.ndarray:
    return data[:, 0] * 604800.0 + data[:, 1]


def to_enu(data: np.ndarray, ref_ecef: np.ndarray) -> np.ndarray:
    ref_pos = rtkcmn.ecef2pos(ref_ecef)
    return np.asarray([rtkcmn.ecef2enu(ref_pos, row[2:5] - ref_ecef) for row in data])


def match_by_time(a: np.ndarray, b: np.ndarray, tolerance: float) -> tuple[np.ndarray, np.ndarray]:
    ta = to_seconds(a)
    tb = to_seconds(b)
    pairs_a = []
    pairs_b = []
    j = 0
    for i, t in enumerate(ta):
        while j + 1 < len(tb) and abs(tb[j + 1] - t) <= abs(tb[j] - t):
            j += 1
        if abs(tb[j] - t) <= tolerance:
            pairs_a.append(i)
            pairs_b.append(j)
    return np.asarray(pairs_a, dtype=int), np.asarray(pairs_b, dtype=int)


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rtk", type=Path, default=repo_root / "docker/output/rtklib.pos")
    parser.add_argument("--fgo", type=Path, default=repo_root / "docker/output/rtklib_fgo.pos")
    parser.add_argument("--save", type=Path, default=repo_root / "docker/output/rtk_fgo_compare.png")
    parser.add_argument(
        "--ref-ecef",
        nargs=3,
        type=float,
        default=[-2267218.0743, 5009581.7561, 3221148.1331],
        metavar=("X", "Y", "Z"),
        help="reference ECEF point used as ENU origin",
    )
    parser.add_argument("--time-tolerance", type=float, default=0.05, help="epoch matching tolerance in seconds")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    ref_ecef = np.asarray(args.ref_ecef, dtype=float)

    rtk = load_pos(args.rtk)
    fgo = load_pos(args.fgo)
    rtk_enu = to_enu(rtk, ref_ecef)
    fgo_enu = to_enu(fgo, ref_ecef)

    idx_rtk, idx_fgo = match_by_time(rtk, fgo, args.time_tolerance)
    if len(idx_rtk) == 0:
        raise SystemExit("no matched epochs found; check time systems or increase --time-tolerance")

    diff = fgo_enu[idx_fgo] - rtk_enu[idx_rtk]
    diff_3d = np.linalg.norm(diff, axis=1)
    rms = np.sqrt(np.mean(diff**2, axis=0))
    rms_3d = np.sqrt(np.mean(diff_3d**2))

    print(f"matched epochs: {len(idx_rtk)}")
    print(f"FGO - RTK RMS East/North/Up: {rms[0]:.3f} {rms[1]:.3f} {rms[2]:.3f} m")
    print(f"FGO - RTK RMS 3D: {rms_3d:.3f} m")

    t0 = min(to_seconds(rtk)[0], to_seconds(fgo)[0])
    rtk_t = to_seconds(rtk) - t0
    fgo_t = to_seconds(fgo) - t0
    match_t = to_seconds(rtk[idx_rtk]) - t0

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    ax_xy, ax_e, ax_n, ax_u = axes.ravel()

    ax_xy.plot(rtk_enu[:, 0], rtk_enu[:, 1], label="RTKLIB RTK", linewidth=1.4)
    ax_xy.plot(fgo_enu[:, 0], fgo_enu[:, 1], label="GLINS FGO", linewidth=1.4)
    ax_xy.scatter(rtk_enu[0, 0], rtk_enu[0, 1], s=30, c="green", label="Start", zorder=3)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.set_xlabel("East (m)")
    ax_xy.set_ylabel("North (m)")
    ax_xy.set_title("ENU Trajectory")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend()

    labels = ("East", "North", "Up")
    for i, ax in enumerate((ax_e, ax_n, ax_u)):
        ax.plot(rtk_t, rtk_enu[:, i], label="RTKLIB RTK", linewidth=1.2)
        ax.plot(fgo_t, fgo_enu[:, i], label="GLINS FGO", linewidth=1.2)
        ax.set_xlabel("Time from start (s)")
        ax.set_ylabel(f"{labels[i]} (m)")
        ax.set_title(f"{labels[i]} Component")
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend()

    fig.suptitle(
        f"RTKLIB vs GLINS FGO | matched {len(idx_rtk)} epochs | "
        f"FGO-RTK RMS ENU=({rms[0]:.2f}, {rms[1]:.2f}, {rms[2]:.2f}) m, 3D={rms_3d:.2f} m"
    )
    fig.tight_layout()

    args.save.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.save, dpi=180, bbox_inches="tight")
    print(f"saved figure: {args.save}")


if __name__ == "__main__":
    main()
