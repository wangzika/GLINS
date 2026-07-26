#!/usr/bin/env python3
"""Evaluate the UrbanNav-HK public ablation runs against SPAN-CPT truth."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


GPS_EPOCH_UNIX = 315964800.0
GPS_UTC_LEAP_SECONDS = 18.0
METHODS = {
    "RTK": ("rtk.pos", "ecef"),
    "RTK/INS": ("rtk_gins/gins.pos", "ecef"),
    "LIO": ("lio/total.pos", "local_lidar"),
    "FM-LiDAR TC": ("fm/fgo.pos", "ecef"),
    "FF-LiDAR TC": ("ff/fgo.pos", "ecef"),
    "GG-LiDAR TC": ("gg/fgo.pos", "ecef"),
    "GG pose-level": ("gg_pose/fgo.pos", "ecef"),
}

# Official public-data calibration, expressed in the UrbanNav body frame.
EXT_GPS = np.array([0.0, 0.86, -0.31])
EXT_LIDAR = np.array([0.0, 0.0, 0.28])
ORIGIN_ECEF = np.array([-2418181.5014, 5385962.2886, 2405305.1800])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_root", type=Path)
    parser.add_argument(
        "--truth",
        type=Path,
        default=Path(
            "/data/zbwang/public/UrbanNav_HK_Medium_20210517/"
            "medium_public_0_120.reference_ecef.csv"
        ),
    )
    return parser.parse_args()


def enu_rotation(latitude_deg: float, longitude_deg: float) -> np.ndarray:
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    return np.array(
        [
            [-math.sin(lon), math.cos(lon), 0.0],
            [
                -math.sin(lat) * math.cos(lon),
                -math.sin(lat) * math.sin(lon),
                math.cos(lat),
            ],
            [
                math.cos(lat) * math.cos(lon),
                math.cos(lat) * math.sin(lon),
                math.sin(lat),
            ],
        ]
    )


def quaternion_rotation(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def load_truth(path: Path) -> dict[str, np.ndarray]:
    rows = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            rows.append(
                [
                    float(row["timestamp"]),
                    float(row["x"]),
                    float(row["y"]),
                    float(row["z"]),
                    float(row["latitude"]),
                    float(row["longitude"]),
                ]
            )
    data = np.asarray(rows)
    return {
        "time": data[:, 0],
        "ecef": data[:, 1:4],
        "latitude": data[:, 4],
        "longitude": data[:, 5],
    }


def load_ecef_solution(path: Path) -> tuple[np.ndarray, np.ndarray, list[int]]:
    times, positions, quality = [], [], []
    with path.open(errors="replace") as stream:
        for raw in stream:
            line = raw.strip()
            if not line or line.startswith("%") or line.startswith("#"):
                continue
            fields = line.split()
            try:
                first = float(fields[0])
                if first > 1.0e9:
                    timestamp = first
                    xyz = [float(value) for value in fields[1:4]]
                    q_index = 4
                else:
                    week = int(float(fields[0]))
                    sow = float(fields[1])
                    timestamp = (
                        GPS_EPOCH_UNIX
                        + week * 604800.0
                        + sow
                        - GPS_UTC_LEAP_SECONDS
                    )
                    xyz = [float(value) for value in fields[2:5]]
                    q_index = 5
                if not np.all(np.isfinite(xyz)):
                    continue
                times.append(timestamp)
                positions.append(xyz)
                quality.append(int(float(fields[q_index])) if len(fields) > q_index else -1)
            except (ValueError, IndexError):
                continue
    return np.asarray(times), np.asarray(positions), quality


def load_local_lio(
    path: Path, ecef_from_enu: np.ndarray
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    times, positions = [], []
    lever_lidar_to_antenna = EXT_GPS - EXT_LIDAR
    with path.open(errors="replace") as stream:
        for raw in stream:
            fields = raw.split()
            if len(fields) < 8:
                continue
            try:
                values = [float(value) for value in fields[:8]]
            except ValueError:
                continue
            timestamp, x, y, z, qx, qy, qz, qw = values
            if timestamp < 1.0e9:
                continue
            rotation = quaternion_rotation(qx, qy, qz, qw)
            antenna_enu = np.array([x, y, z]) + rotation @ lever_lidar_to_antenna
            antenna_ecef = ORIGIN_ECEF + ecef_from_enu @ antenna_enu
            times.append(timestamp)
            positions.append(antenna_ecef)
    return np.asarray(times), np.asarray(positions), [-1] * len(times)


def interpolate_truth(
    estimate_time: np.ndarray, truth_time: np.ndarray, truth_ecef: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    valid = (estimate_time >= truth_time[0]) & (estimate_time <= truth_time[-1])
    selected_time = estimate_time[valid]
    interpolated = np.column_stack(
        [np.interp(selected_time, truth_time, truth_ecef[:, axis]) for axis in range(3)]
    )
    return valid, interpolated


def calculate_metrics(errors_enu: np.ndarray) -> dict[str, float | int]:
    error_3d = np.linalg.norm(errors_enu, axis=1)
    rms_axes = np.sqrt(np.mean(np.square(errors_enu), axis=0))
    return {
        "matched_epochs": int(len(error_3d)),
        "rms_e_m": float(rms_axes[0]),
        "rms_n_m": float(rms_axes[1]),
        "rms_u_m": float(rms_axes[2]),
        "rms_3d_m": float(np.sqrt(np.mean(np.square(error_3d)))),
        "median_3d_m": float(np.median(error_3d)),
        "p95_3d_m": float(np.percentile(error_3d, 95)),
        "max_3d_m": float(np.max(error_3d)),
        "availability_lt_0_3m_pct": float(np.mean(error_3d < 0.3) * 100.0),
        "availability_lt_1m_pct": float(np.mean(error_3d < 1.0) * 100.0),
    }


def read_status(path: Path) -> dict[str, int | float]:
    result: dict[str, int | float] = {}
    if not path.exists():
        return result
    for line in path.read_text(errors="replace").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        try:
            result[key] = int(value)
        except ValueError:
            try:
                result[key] = float(value)
            except ValueError:
                pass
    return result


def main() -> int:
    args = parse_args()
    result_root = args.result_root.resolve()
    figure_dir = result_root / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)

    truth = load_truth(args.truth)
    rotation_enu_from_ecef = enu_rotation(
        truth["latitude"][0], truth["longitude"][0]
    )
    rotation_ecef_from_enu = rotation_enu_from_ecef.T
    truth_enu = (rotation_enu_from_ecef @ (truth["ecef"] - ORIGIN_ECEF).T).T

    all_results: dict[str, dict] = {}
    plotted: dict[str, dict[str, np.ndarray]] = {}
    for method, (relative_path, solution_type) in METHODS.items():
        solution_path = result_root / relative_path
        if not solution_path.exists() or solution_path.stat().st_size == 0:
            continue
        if solution_type == "local_lidar":
            estimate_time, estimate_ecef, quality = load_local_lio(
                solution_path, rotation_ecef_from_enu
            )
        else:
            estimate_time, estimate_ecef, quality = load_ecef_solution(solution_path)
        if len(estimate_time) == 0:
            continue
        valid, interpolated_truth = interpolate_truth(
            estimate_time, truth["time"], truth["ecef"]
        )
        estimate_time = estimate_time[valid]
        estimate_ecef = estimate_ecef[valid]
        quality_array = np.asarray(quality)[valid]
        if len(estimate_time) == 0:
            continue
        errors_enu = (
            rotation_enu_from_ecef @ (estimate_ecef - interpolated_truth).T
        ).T
        estimate_enu = (
            rotation_enu_from_ecef @ (estimate_ecef - ORIGIN_ECEF).T
        ).T
        metrics = calculate_metrics(errors_enu)
        metrics["solution_epochs"] = int(len(estimate_time))
        metrics["q1_fixed"] = int(np.sum(quality_array == 1))
        metrics["q2_float"] = int(np.sum(quality_array == 2))
        metrics["q4_dgps"] = int(np.sum(quality_array == 4))
        status_name = {
            "RTK/INS": "rtk_gins",
            "LIO": "lio",
            "FM-LiDAR TC": "fm",
            "FF-LiDAR TC": "ff",
            "GG-LiDAR TC": "gg",
            "GG pose-level": "gg_pose",
        }.get(method)
        if status_name:
            metrics.update(
                {f"run_{key}": value for key, value in read_status(
                    result_root / status_name / "status.txt"
                ).items()}
            )
        all_results[method] = metrics
        plotted[method] = {
            "time": estimate_time,
            "enu": estimate_enu,
            "error_enu": errors_enu,
        }

    if not all_results:
        raise RuntimeError(f"No readable solutions found under {result_root}")

    metric_columns = [
        "matched_epochs",
        "rms_e_m",
        "rms_n_m",
        "rms_u_m",
        "rms_3d_m",
        "median_3d_m",
        "p95_3d_m",
        "max_3d_m",
        "availability_lt_0_3m_pct",
        "availability_lt_1m_pct",
        "q1_fixed",
        "q2_float",
        "q4_dgps",
        "run_elapsed_s",
        "run_rollbacks",
        "run_lidar_rejects",
    ]
    with (result_root / "metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["method", *metric_columns])
        writer.writeheader()
        for method, metrics in all_results.items():
            writer.writerow(
                {"method": method, **{key: metrics.get(key, "") for key in metric_columns}}
            )
    (result_root / "metrics.json").write_text(
        json.dumps(all_results, indent=2, ensure_ascii=False) + "\n"
    )

    colors = plt.get_cmap("tab10")
    color_for = {method: colors(index) for index, method in enumerate(all_results)}

    fig, ax = plt.subplots(figsize=(9.2, 7.2))
    ax.plot(truth_enu[:, 0], truth_enu[:, 1], "k-", linewidth=3, label="SPAN-CPT truth")
    for method, data in plotted.items():
        ax.plot(
            data["enu"][:, 0],
            data["enu"][:, 1],
            linewidth=1.6,
            color=color_for[method],
            label=method,
        )
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("UrbanNav-HK Medium: horizontal trajectories")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(figure_dir / "trajectory_enu.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10.5, 6.0))
    for method, data in plotted.items():
        elapsed = data["time"] - truth["time"][0]
        error_3d = np.linalg.norm(data["error_enu"], axis=1)
        ax.plot(elapsed, error_3d, linewidth=1.4, color=color_for[method], label=method)
    ax.axhline(0.3, color="0.35", linestyle="--", linewidth=1, label="0.3 m")
    ax.axhline(1.0, color="0.55", linestyle=":", linewidth=1, label="1.0 m")
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("3D position error (m)")
    ax.set_title("3D error over time")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(figure_dir / "error_3d_timeseries.png", dpi=180)
    plt.close(fig)

    stable_methods = [
        method for method in all_results if all_results[method]["rms_3d_m"] < 50.0
    ]
    if stable_methods:
        fig, ax = plt.subplots(figsize=(10.5, 6.0))
        for method in stable_methods:
            data = plotted[method]
            elapsed = data["time"] - truth["time"][0]
            error_3d = np.linalg.norm(data["error_enu"], axis=1)
            ax.plot(
                elapsed,
                error_3d,
                linewidth=1.7,
                color=color_for[method],
                label=method,
            )
        ax.axhline(0.3, color="0.35", linestyle="--", linewidth=1, label="0.3 m")
        ax.axhline(1.0, color="0.55", linestyle=":", linewidth=1, label="1.0 m")
        ax.set_xlabel("Elapsed time (s)")
        ax.set_ylabel("3D position error (m)")
        ax.set_title("3D error over time: non-divergent methods")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9, ncol=2)
        fig.tight_layout()
        fig.savefig(figure_dir / "error_3d_stable_zoom.png", dpi=180)
        plt.close(fig)

    methods = list(all_results)
    x = np.arange(len(methods))
    width = 0.19
    fig, ax = plt.subplots(figsize=(11.2, 6.2))
    for index, (key, label) in enumerate(
        [("rms_e_m", "E"), ("rms_n_m", "N"), ("rms_u_m", "U"), ("rms_3d_m", "3D")]
    ):
        values = [all_results[method][key] for method in methods]
        ax.bar(x + (index - 1.5) * width, values, width, label=label)
    ax.set_xticks(x)
    ax.set_xticklabels(methods, rotation=18, ha="right")
    ax.set_ylabel("RMS error (m)")
    ax.set_title("Position RMS comparison")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(figure_dir / "rms_comparison.png", dpi=180)
    plt.close(fig)

    if stable_methods:
        stable_x = np.arange(len(stable_methods))
        fig, ax = plt.subplots(figsize=(9.8, 6.0))
        for index, (key, label) in enumerate(
            [
                ("rms_e_m", "E"),
                ("rms_n_m", "N"),
                ("rms_u_m", "U"),
                ("rms_3d_m", "3D"),
            ]
        ):
            values = [all_results[method][key] for method in stable_methods]
            ax.bar(
                stable_x + (index - 1.5) * width,
                values,
                width,
                label=label,
            )
        ax.set_xticks(stable_x)
        ax.set_xticklabels(stable_methods, rotation=15, ha="right")
        ax.set_ylabel("RMS error (m)")
        ax.set_title("Position RMS: non-divergent methods")
        ax.grid(axis="y", alpha=0.25)
        ax.legend()
        fig.tight_layout()
        fig.savefig(figure_dir / "rms_stable_zoom.png", dpi=180)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(10.4, 5.8))
    availability_03 = [
        all_results[method]["availability_lt_0_3m_pct"] for method in methods
    ]
    availability_1 = [
        all_results[method]["availability_lt_1m_pct"] for method in methods
    ]
    ax.bar(x - 0.2, availability_03, 0.4, label="3D error < 0.3 m")
    ax.bar(x + 0.2, availability_1, 0.4, label="3D error < 1.0 m")
    ax.set_xticks(x)
    ax.set_xticklabels(methods, rotation=18, ha="right")
    ax.set_ylim(0, 105)
    ax.set_ylabel("Availability (%)")
    ax.set_title("Accuracy availability")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(figure_dir / "availability.png", dpi=180)
    plt.close(fig)

    print(json.dumps(all_results, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
