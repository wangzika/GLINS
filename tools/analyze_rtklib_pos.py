#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import statistics
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
GNSS_DIR = ROOT / "数据" / "gnss"
POS_PATH = GNSS_DIR / "rtklib.pos"
DOP_PATH = GNSS_DIR / "dop.txt"
SVG_PATH = GNSS_DIR / "rtk_pose_quality.svg"
JSON_PATH = GNSS_DIR / "rtk_quality_metrics.json"


@dataclass
class PosRecord:
    week: int
    tow: float
    x: float
    y: float
    z: float
    q: int
    ns: int
    sdx: float
    sdy: float
    sdz: float
    sdxy: float
    sdyz: float
    sdzx: float
    age: float
    ratio: float
    e: float = 0.0
    n: float = 0.0
    u: float = 0.0


@dataclass
class DopRecord:
    time_text: str
    nsat: int
    gdop: float
    pdop: float
    hdop: float
    vdop: float


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    values = sorted(values)
    idx = (len(values) - 1) * pct
    lo = math.floor(idx)
    hi = math.ceil(idx)
    if lo == hi:
        return values[lo]
    frac = idx - lo
    return values[lo] * (1.0 - frac) + values[hi] * frac


def parse_pos(path: Path) -> tuple[dict[str, str], list[PosRecord]]:
    header: dict[str, str] = {}
    records: list[PosRecord] = []
    with path.open(encoding="utf-8", errors="ignore") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("%"):
                if ":" in line:
                    key, value = line[1:].split(":", 1)
                    header[key.strip()] = value.strip()
                continue
            parts = line.split()
            if len(parts) < 15:
                continue
            records.append(
                PosRecord(
                    week=int(parts[0]),
                    tow=float(parts[1]),
                    x=float(parts[2]),
                    y=float(parts[3]),
                    z=float(parts[4]),
                    q=int(parts[5]),
                    ns=int(parts[6]),
                    sdx=float(parts[7]),
                    sdy=float(parts[8]),
                    sdz=float(parts[9]),
                    sdxy=float(parts[10]),
                    sdyz=float(parts[11]),
                    sdzx=float(parts[12]),
                    age=float(parts[13]),
                    ratio=float(parts[14]),
                )
            )
    return header, records


def parse_dop(path: Path) -> list[DopRecord]:
    rows: list[DopRecord] = []
    if not path.exists():
        return rows
    with path.open(encoding="utf-8", errors="ignore") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 10:
                continue
            rows.append(
                DopRecord(
                    time_text=f"{parts[0]} {parts[1]}",
                    nsat=int(parts[2]),
                    gdop=float(parts[-4]),
                    pdop=float(parts[-3]),
                    hdop=float(parts[-2]),
                    vdop=float(parts[-1]),
                )
            )
    return rows


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    a = 6378137.0
    e2 = 6.69437999014e-3
    b = a * math.sqrt(1.0 - e2)
    ep2 = (a * a - b * b) / (b * b)
    p = math.hypot(x, y)
    th = math.atan2(a * z, b * p)
    lon = math.atan2(y, x)
    lat = math.atan2(z + ep2 * b * math.sin(th) ** 3, p - e2 * a * math.cos(th) ** 3)
    n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - n
    return lat, lon, h


def ecef_to_enu(x: float, y: float, z: float, ref_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    xr, yr, zr = ref_xyz
    lat, lon, _ = ecef_to_geodetic(xr, yr, zr)
    dx, dy, dz = x - xr, y - yr, z - zr
    slat, clat = math.sin(lat), math.cos(lat)
    slon, clon = math.sin(lon), math.cos(lon)
    e = -slon * dx + clon * dy
    n = -slat * clon * dx - slat * slon * dy + clat * dz
    u = clat * clon * dx + clat * slon * dy + slat * dz
    return e, n, u


def run_lengths(states: Iterable[int], target: int) -> list[int]:
    runs: list[int] = []
    cur = 0
    for s in states:
        if s == target:
            cur += 1
        elif cur:
            runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)
    return runs


def track_length(records: list[PosRecord]) -> float:
    total = 0.0
    for prev, cur in zip(records, records[1:]):
        de = cur.e - prev.e
        dn = cur.n - prev.n
        du = cur.u - prev.u
        total += math.sqrt(de * de + dn * dn + du * du)
    return total


def axis_ranges(records: list[PosRecord]) -> dict[str, float]:
    east = [r.e for r in records]
    north = [r.n for r in records]
    up = [r.u for r in records]
    return {
        "east_min": min(east),
        "east_max": max(east),
        "north_min": min(north),
        "north_max": max(north),
        "up_min": min(up),
        "up_max": max(up),
    }


def make_svg(records: list[PosRecord], out_path: Path) -> None:
    width = 1200
    height = 900
    margin = 80
    plot_w = width - 2 * margin
    plot_h = 560
    time_h = 120

    east = [r.e for r in records]
    north = [r.n for r in records]
    times = [r.tow for r in records]
    ratios = [r.ratio for r in records]

    min_e, max_e = min(east), max(east)
    min_n, max_n = min(north), max(north)
    if abs(max_e - min_e) < 1e-6:
        max_e += 1.0
        min_e -= 1.0
    if abs(max_n - min_n) < 1e-6:
        max_n += 1.0
        min_n -= 1.0

    colors = {
        1: "#157f1f",
        2: "#f39c12",
        3: "#3498db",
        4: "#8e44ad",
        5: "#7f8c8d",
        6: "#c0392b",
    }

    def map_xy(e: float, n: float) -> tuple[float, float]:
        x = margin + (e - min_e) / (max_e - min_e) * plot_w
        y = margin + plot_h - (n - min_n) / (max_n - min_n) * plot_h
        return x, y

    min_t, max_t = min(times), max(times)
    if abs(max_t - min_t) < 1e-6:
        max_t += 1.0

    def map_t(t: float) -> float:
        return margin + (t - min_t) / (max_t - min_t) * plot_w

    max_ratio = max(max(ratios), 1.0)
    ratio_h = 80
    ratio_y0 = margin + plot_h + 150

    def map_ratio(r: float) -> float:
        return ratio_y0 + ratio_h - min(r, max_ratio) / max_ratio * ratio_h

    lines: list[str] = []
    lines.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">')
    lines.append('<rect width="100%" height="100%" fill="#f8f9fb"/>')
    lines.append('<text x="80" y="40" font-size="28" font-family="monospace" fill="#1f2937">RTK Pose Quality Overview</text>')
    lines.append('<text x="80" y="66" font-size="14" font-family="monospace" fill="#4b5563">Trajectory in local ENU, solution-state timeline, and ambiguity ratio</text>')

    lines.append(f'<rect x="{margin}" y="{margin}" width="{plot_w}" height="{plot_h}" fill="white" stroke="#cbd5e1"/>')
    for tick in range(6):
        tx = margin + tick * plot_w / 5.0
        ty = margin + tick * plot_h / 5.0
        lines.append(f'<line x1="{tx:.1f}" y1="{margin}" x2="{tx:.1f}" y2="{margin + plot_h}" stroke="#e5e7eb"/>')
        lines.append(f'<line x1="{margin}" y1="{ty:.1f}" x2="{margin + plot_w}" y2="{ty:.1f}" stroke="#e5e7eb"/>')

    prev = None
    for rec in records:
        x, y = map_xy(rec.e, rec.n)
        if prev is not None:
            px, py = map_xy(prev.e, prev.n)
            lines.append(f'<line x1="{px:.2f}" y1="{py:.2f}" x2="{x:.2f}" y2="{y:.2f}" stroke="#cbd5e1" stroke-width="1"/>')
        prev = rec

    for rec in records:
        x, y = map_xy(rec.e, rec.n)
        color = colors.get(rec.q, "#111827")
        radius = 3.2 if rec.q == 1 else 2.3
        lines.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{radius}" fill="{color}" fill-opacity="0.88"/>')

    start_x, start_y = map_xy(records[0].e, records[0].n)
    end_x, end_y = map_xy(records[-1].e, records[-1].n)
    lines.append(f'<circle cx="{start_x:.2f}" cy="{start_y:.2f}" r="5" fill="#111827"/>')
    lines.append(f'<text x="{start_x + 8:.2f}" y="{start_y - 8:.2f}" font-size="12" font-family="monospace" fill="#111827">start</text>')
    lines.append(f'<circle cx="{end_x:.2f}" cy="{end_y:.2f}" r="5" fill="#b91c1c"/>')
    lines.append(f'<text x="{end_x + 8:.2f}" y="{end_y - 8:.2f}" font-size="12" font-family="monospace" fill="#b91c1c">end</text>')

    lines.append(f'<text x="{margin}" y="{margin + plot_h + 24}" font-size="13" font-family="monospace" fill="#374151">East (m): {min_e:.1f} .. {max_e:.1f}</text>')
    lines.append(f'<text x="{margin + 260}" y="{margin + plot_h + 24}" font-size="13" font-family="monospace" fill="#374151">North (m): {min_n:.1f} .. {max_n:.1f}</text>')

    timeline_y = margin + plot_h + 50
    lines.append(f'<rect x="{margin}" y="{timeline_y}" width="{plot_w}" height="{time_h}" fill="white" stroke="#cbd5e1"/>')
    lines.append(f'<text x="{margin}" y="{timeline_y - 10}" font-size="14" font-family="monospace" fill="#1f2937">Solution state timeline</text>')
    for rec in records:
        x = map_t(rec.tow)
        color = colors.get(rec.q, "#111827")
        lines.append(f'<line x1="{x:.2f}" y1="{timeline_y}" x2="{x:.2f}" y2="{timeline_y + time_h}" stroke="{color}" stroke-width="1.2"/>')

    lines.append(f'<rect x="{margin}" y="{ratio_y0}" width="{plot_w}" height="{ratio_h}" fill="white" stroke="#cbd5e1"/>')
    lines.append(f'<text x="{margin}" y="{ratio_y0 - 10}" font-size="14" font-family="monospace" fill="#1f2937">Ambiguity ratio</text>')
    for prev, rec in zip(records, records[1:]):
        x1, y1 = map_t(prev.tow), map_ratio(prev.ratio)
        x2, y2 = map_t(rec.tow), map_ratio(rec.ratio)
        lines.append(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="#2563eb" stroke-width="1.5"/>')

    legend_x = width - 250
    legend_y = 105
    lines.append(f'<rect x="{legend_x}" y="{legend_y}" width="170" height="154" rx="8" fill="white" stroke="#cbd5e1"/>')
    lines.append(f'<text x="{legend_x + 14}" y="{legend_y + 22}" font-size="14" font-family="monospace" fill="#111827">Q status legend</text>')
    labels = {
        1: "1 fix",
        2: "2 float",
        3: "3 sbas",
        4: "4 dgps",
        5: "5 single",
        6: "6 ppp",
    }
    y_cursor = legend_y + 44
    for q in [1, 2, 3, 4, 5, 6]:
        color = colors.get(q, "#111827")
        lines.append(f'<circle cx="{legend_x + 18}" cy="{y_cursor - 4}" r="5" fill="{color}"/>')
        lines.append(f'<text x="{legend_x + 32}" y="{y_cursor}" font-size="13" font-family="monospace" fill="#374151">{labels[q]}</text>')
        y_cursor += 20

    for tick in range(6):
        t = min_t + tick * (max_t - min_t) / 5.0
        x = map_t(t)
        label = f"{t:.0f}"
        lines.append(f'<text x="{x - 18:.2f}" y="{timeline_y + time_h + 20}" font-size="11" font-family="monospace" fill="#6b7280">{label}</text>')

    lines.append("</svg>")
    out_path.write_text("\n".join(lines), encoding="utf-8")


def summarize(records: list[PosRecord], dop_records: list[DopRecord], header: dict[str, str]) -> dict[str, object]:
    ref_text = header.get("ref pos", "")
    ref_xyz = tuple(float(v) for v in ref_text.split()[:3])
    for rec in records:
        rec.e, rec.n, rec.u = ecef_to_enu(rec.x, rec.y, rec.z, ref_xyz)

    q_counts = Counter(rec.q for rec in records)
    fixed = [rec for rec in records if rec.q == 1]
    float_solutions = [rec for rec in records if rec.q == 2]
    any_diff = [rec for rec in records if rec.q in {1, 2, 4}]
    fixed_runs = run_lengths((rec.q for rec in records), 1)
    float_runs = run_lengths((rec.q for rec in records), 2)

    times = [rec.tow for rec in records]
    start_tow, end_tow = min(times), max(times)
    first_fix_tow = next((rec.tow for rec in records if rec.q == 1), None)

    enu_ranges = axis_ranges(records)
    fix_ratios = [rec.ratio for rec in fixed]
    float_ratios = [rec.ratio for rec in float_solutions]
    route_length = track_length(records)

    dop_pdop = [d.pdop for d in dop_records]
    dop_hdop = [d.hdop for d in dop_records]
    dop_vdop = [d.vdop for d in dop_records]
    dop_nsat = [d.nsat for d in dop_records]

    return {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "input_pos": str(POS_PATH),
        "input_dop": str(DOP_PATH),
        "epochs": len(records),
        "start_tow": start_tow,
        "end_tow": end_tow,
        "duration_s": end_tow - start_tow,
        "first_fix_delay_s": None if first_fix_tow is None else first_fix_tow - start_tow,
        "q_counts": dict(sorted(q_counts.items())),
        "fix_rate": len(fixed) / len(records),
        "float_rate": len(float_solutions) / len(records),
        "diff_rate": len(any_diff) / len(records),
        "longest_fix_run_s": max(fixed_runs) if fixed_runs else 0,
        "longest_float_run_s": max(float_runs) if float_runs else 0,
        "avg_ns": statistics.fmean(rec.ns for rec in records),
        "min_ns": min(rec.ns for rec in records),
        "max_ns": max(rec.ns for rec in records),
        "median_ratio_fix": statistics.median(fix_ratios) if fix_ratios else None,
        "p95_ratio_fix": percentile(fix_ratios, 0.95) if fix_ratios else None,
        "median_ratio_float": statistics.median(float_ratios) if float_ratios else None,
        "avg_age": statistics.fmean(rec.age for rec in records),
        "route_length_m": route_length,
        "enu_ranges": enu_ranges,
        "max_horizontal_offset_from_ref_m": max(math.hypot(rec.e, rec.n) for rec in records),
        "max_abs_up_m": max(abs(rec.u) for rec in records),
        "fix_sigma_median": {
            "sdx": statistics.median(rec.sdx for rec in fixed) if fixed else None,
            "sdy": statistics.median(rec.sdy for rec in fixed) if fixed else None,
            "sdz": statistics.median(rec.sdz for rec in fixed) if fixed else None,
        },
        "float_sigma_median": {
            "sdx": statistics.median(rec.sdx for rec in float_solutions) if float_solutions else None,
            "sdy": statistics.median(rec.sdy for rec in float_solutions) if float_solutions else None,
            "sdz": statistics.median(rec.sdz for rec in float_solutions) if float_solutions else None,
        },
        "dop": {
            "epochs": len(dop_records),
            "avg_nsat": statistics.fmean(dop_nsat) if dop_nsat else None,
            "min_nsat": min(dop_nsat) if dop_nsat else None,
            "max_nsat": max(dop_nsat) if dop_nsat else None,
            "median_pdop": statistics.median(dop_pdop) if dop_pdop else None,
            "p95_pdop": percentile(dop_pdop, 0.95) if dop_pdop else None,
            "median_hdop": statistics.median(dop_hdop) if dop_hdop else None,
            "median_vdop": statistics.median(dop_vdop) if dop_vdop else None,
        },
        "reference_ecef_m": ref_xyz,
    }


def main() -> None:
    header, records = parse_pos(POS_PATH)
    dop_records = parse_dop(DOP_PATH)
    metrics = summarize(records, dop_records, header)
    JSON_PATH.write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")
    make_svg(records, SVG_PATH)
    print(f"metrics: {JSON_PATH}")
    print(f"plot: {SVG_PATH}")


if __name__ == "__main__":
    main()
