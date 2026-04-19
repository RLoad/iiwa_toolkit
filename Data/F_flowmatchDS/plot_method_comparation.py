#!/usr/bin/env python3
"""
Visualize FlowMatch / phase-2 recordings and compute paper-style metrics.

Expects phase2_<method>.txt next to optional phase2_*_target_area.ply (ASCII x,y,z,r,g,b)
and optional phase2_<method>_waypoints.txt (copy of the trajectory file at phase-2 entry).

Metrics (aligned with paper definitions where data allows):

- **CR (coverage rate)** — Full definition needs a removal mask Omega_removed from imagery.
  Here we report **CR_proxy** = |Omega_cover ∩ Omega_target| / |Omega_target| with
  Omega_cover a tube of radius `--tool-radius` around the projected EE path on the target
  PCA plane, and Omega_target the convex hull of the target cloud (2D area).

- **OPR (over-polish rate)** — |Omega_cover \\ Omega_target| / |Omega_target| on the same plane.

- **CU (coverage uniformity)** — Target hull partitioned into a regular grid (`--cu-grid`^2 cells
  with centers inside hull); n_k = dwell counts of path samples per cell; CU = std(n)/mean(n).

- **RSR / T_rec** — Optional `--recovery-json`: a JSON list of events with the same time base
  as phase2 (ROS wall time in seconds), e.g.::
    [
      {"t_disturb": 1776599600.0, "t_return": 1776599601.2, "success": true}
    ]
  RSR = N_succ/N_dist * 100%; T_rec per event is t_return - t_disturb.

- **e_F^RMS** — sqrt(mean((F_n(t) - F_n*(t))^2)) with F_n = F_real·n, F_n* = F_des·n, n the
  target-plane normal from PCA (sign aligned with mean measured force). If no PLY, falls back
  to Fz vs desired Fz as a rough proxy.

Outputs: two figures when using `--output base.png` → base_forces.png, base_cloud.png.

Usage:
  python3 plot_method_comparation.py [--runs ...] [--output fig.png] [--recovery-json ev.json]
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib.pyplot as plt
    from matplotlib.path import Path as MplPath
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
except ImportError as e:
    print("This script needs matplotlib. Install with: pip install matplotlib", file=sys.stderr)
    raise SystemExit(1) from e


# ---------------------------------------------------------------------------
# PLY (ASCII)
# ---------------------------------------------------------------------------


def load_ascii_ply(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return xyz (N,3) float, rgb (N,3) uint8 or zeros."""
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    if not lines or lines[0].strip() != "ply":
        raise ValueError(f"Not an ASCII PLY: {path}")
    i = 1
    n_vert = 0
    props: list[str] = []
    while i < len(lines):
        line = lines[i].strip()
        i += 1
        if line.startswith("element vertex"):
            n_vert = int(line.split()[-1])
        elif line.startswith("property"):
            props.append(line.split()[-1])
        elif line == "end_header":
            break
    data_lines = lines[i : i + n_vert]
    if len(data_lines) < n_vert:
        raise ValueError(f"Incomplete PLY body: {path}")
    xyz = np.zeros((n_vert, 3), dtype=np.float64)
    rgb = np.zeros((n_vert, 3), dtype=np.uint8)
    has_rgb = all(p in props for p in ("red", "green", "blue"))
    ix, iy, iz = props.index("x"), props.index("y"), props.index("z")
    ir, ig, ib = (props.index("red"), props.index("green"), props.index("blue")) if has_rgb else (0, 0, 0)
    for j, dl in enumerate(data_lines):
        parts = dl.split()
        if len(parts) < 3:
            continue
        xyz[j, 0] = float(parts[ix])
        xyz[j, 1] = float(parts[iy])
        xyz[j, 2] = float(parts[iz])
        if has_rgb and len(parts) > max(ir, ig, ib):
            rgb[j, 0] = int(float(parts[ir]))
            rgb[j, 1] = int(float(parts[ig]))
            rgb[j, 2] = int(float(parts[ib]))
    return xyz, rgb


def find_target_ply(phase2_txt: Path) -> Path | None:
    """Sibling file phase2_*_target_area.ply."""
    parent = phase2_txt.parent
    cands = sorted(parent.glob("*_target_area.ply"))
    if not cands:
        cands = sorted(parent.glob("*.ply"))
    return cands[0] if cands else None


def load_waypoints_txt(path: Path) -> np.ndarray:
    """Lines of x y z (floats); skip blanks and # comments."""
    rows: list[list[float]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 3:
            continue
        try:
            rows.append([float(parts[0]), float(parts[1]), float(parts[2])])
        except ValueError:
            continue
    return np.asarray(rows, dtype=np.float64)


def find_waypoints_txt(phase2_txt: Path) -> Path | None:
    """Sibling file copied at phase-2 entry: phase2_<method>_waypoints.txt."""
    parent = phase2_txt.parent
    cands = sorted(parent.glob("phase2_*_waypoints.txt"))
    return cands[0] if cands else None


# ---------------------------------------------------------------------------
# Geometry: PCA plane, 2D hull, raster
# ---------------------------------------------------------------------------


def pca_plane_basis(xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """center (3,), normal (3,), u (3,), v (3,) orthonormal on plane."""
    c = xyz.mean(axis=0)
    x = xyz - c
    _, _, vt = np.linalg.svd(x, full_matrices=True)
    n = vt[-1, :].astype(np.float64)
    n = n / (np.linalg.norm(n) + 1e-15)
    # arbitrary u ⟂ n
    t = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(np.dot(t, n)) > 0.9:
        t = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    u = np.cross(n, t)
    u = u / (np.linalg.norm(u) + 1e-15)
    v = np.cross(n, u)
    v = v / (np.linalg.norm(v) + 1e-15)
    return c, n, u, v


def project_uv(xyz: np.ndarray, c: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    d = xyz - c
    return np.stack([d @ u, d @ v], axis=1)


def convex_hull_2d_mono(points: np.ndarray) -> np.ndarray:
    """Monotone chain; points (N,2), returns hull vertices CCW without duplicate closing."""
    if len(points) < 3:
        return points
    pts = np.unique(points.round(decimals=9), axis=0)
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[np.ndarray] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[np.ndarray] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = np.array(lower[:-1] + upper[:-1], dtype=np.float64)
    return hull


def polygon_area(poly: np.ndarray) -> float:
    if len(poly) < 3:
        return 0.0
    x = poly[:, 0]
    y = poly[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def point_in_polygon(pt: np.ndarray, poly: np.ndarray) -> bool:
    x, y = pt[0], pt[1]
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-30) + xi):
            inside = not inside
        j = i
    return inside


def densify_polyline_2d(path_uv: np.ndarray, max_step_m: float) -> np.ndarray:
    """Sample points along segments so consecutive points are at most max_step_m apart."""
    if len(path_uv) < 2:
        return path_uv.copy()
    out: list[np.ndarray] = [path_uv[0].copy()]
    for i in range(len(path_uv) - 1):
        a = path_uv[i]
        b = path_uv[i + 1]
        seg = b - a
        L = float(np.linalg.norm(seg))
        if L < 1e-12:
            continue
        n = max(1, int(math.ceil(L / max_step_m)))
        for j in range(1, n + 1):
            out.append(a + seg * (j / n))
    return np.asarray(out, dtype=np.float64)


def min_dist_to_path_grid(
    centers: np.ndarray,
    path_uv: np.ndarray,
) -> np.ndarray:
    """Min distance from each center (N,2) to polyline; uses scipy cKDTree on densified path."""
    dense = densify_polyline_2d(path_uv, max_step_m=max(0.0002, 1e-6))
    try:
        from scipy.spatial import cKDTree

        tree = cKDTree(dense)
        d, _ = tree.query(centers)
        return np.asarray(d, dtype=np.float64)
    except Exception:
        # Chunked brute force (no scipy): avoid allocating full (N x M) matrix
        out = np.empty(len(centers), dtype=np.float64)
        chunk = 2048
        for i0 in range(0, len(centers), chunk):
            sl = slice(i0, min(i0 + chunk, len(centers)))
            d2 = np.min(
                np.sum((centers[sl, None, :] - dense[None, :, :]) ** 2, axis=2),
                axis=1,
            )
            out[sl] = np.sqrt(np.maximum(d2, 0.0))
        return out


def raster_coverage_metrics(
    target_uv: np.ndarray,
    path_uv: np.ndarray,
    tool_radius_m: float,
    grid: int,
) -> tuple[float, float, float, float]:
    """
    Returns (area_target, area_cover_in_target, area_cover_outside, cell_m).
    Proxy for CR/OPR: cover = tube around path of radius tool_radius_m.
    """
    hull = convex_hull_2d_mono(target_uv)
    if len(hull) < 3:
        return 0.0, 0.0, 0.0, 1.0

    pad = max(tool_radius_m * 3, 1e-4)
    umin, umax = hull[:, 0].min() - pad, hull[:, 0].max() + pad
    vmin, vmax = hull[:, 1].min() - pad, hull[:, 1].max() + pad
    du = (umax - umin) / grid
    dv = (vmax - vmin) / grid
    cell_area = abs(du * dv)

    nu = nv = grid
    uc = umin + (np.arange(nu) + 0.5) * du
    vc = vmin + (np.arange(nv) + 0.5) * dv
    uu, vv = np.meshgrid(uc, vc, indexing="ij")
    centers = np.stack([uu.ravel(), vv.ravel()], axis=1)
    dists = min_dist_to_path_grid(centers, path_uv)
    poly = MplPath(np.vstack([hull, hull[0]]))
    in_hull = poly.contains_points(centers)
    cover = dists <= tool_radius_m

    at = float(np.sum(in_hull) * cell_area)
    ac_in = float(np.sum(in_hull & cover) * cell_area)
    ac_out = float(np.sum((~in_hull) & cover) * cell_area)

    return at, ac_in, ac_out, math.sqrt(cell_area)


def coverage_uniformity(
    path_uv: np.ndarray,
    hull: np.ndarray,
    grid_k: int,
) -> tuple[float, np.ndarray]:
    """
    grid_k^2 equal-area cells on bbox of hull; dwell counts n_k for trajectory samples inside hull.
    Returns CU = std(n)/mean(n), and n_k (cells whose center lies in hull).
    """
    pad = 1e-5
    umin, umax = hull[:, 0].min() - pad, hull[:, 0].max() + pad
    vmin, vmax = hull[:, 1].min() - pad, hull[:, 1].max() + pad
    du = (umax - umin) / grid_k
    dv = (vmax - vmin) / grid_k
    counts: dict[tuple[int, int], int] = {}
    for p in path_uv:
        if not point_in_polygon(p, hull):
            continue
        iu = int(np.clip((p[0] - umin) / du, 0, grid_k - 1e-9))
        iv = int(np.clip((p[1] - vmin) / dv, 0, grid_k - 1e-9))
        key = (iu, iv)
        counts[key] = counts.get(key, 0) + 1
    # include zero-count cells inside hull (equal-area grid)
    nk: list[int] = []
    for iu in range(grid_k):
        for iv in range(grid_k):
            uc = umin + (iu + 0.5) * du
            vc = vmin + (iv + 0.5) * dv
            center = np.array([uc, vc], dtype=np.float64)
            if not point_in_polygon(center, hull):
                continue
            nk.append(counts.get((iu, iv), 0))
    arr = np.array(nk, dtype=np.float64)
    if arr.size == 0 or arr.mean() < 1e-12:
        return float("nan"), arr
    cu = float(arr.std() / arr.mean())
    return cu, arr


def normal_force_series(
    F: np.ndarray, n: np.ndarray
) -> np.ndarray:
    """Scalar normal component; flip sign so typical contact is positive."""
    fn = F @ n
    if np.nanmean(fn) < 0:
        n = -n
        fn = F @ n
    return fn


def parse_phase2_txt(path: Path) -> tuple[str, np.ndarray]:
    method = path.stem.replace("phase2_", "", 1)
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                m = re.match(r"#\s*method\s+(.+)", line)
                if m:
                    method = m.group(1).strip()
                continue
            parts = line.split()
            if len(parts) < 14:
                continue
            try:
                rows.append([float(x) for x in parts[:14]])
            except ValueError:
                continue
    if not rows:
        return method, np.zeros((0, 14))
    return method, np.asarray(rows, dtype=np.float64)


def discover_phase2_files(base: Path, run_filter: list[str] | None) -> list[Path]:
    out: list[Path] = []
    if run_filter:
        for name in run_filter:
            d = base / name
            if not d.is_dir():
                print(f"Warning: run folder not found: {d}", file=sys.stderr)
                continue
            out.extend(sorted(d.glob("phase2_*.txt")))
        return sorted(out)

    for sub in sorted(base.iterdir()):
        if not sub.is_dir():
            continue
        if sub.name.startswith("."):
            continue
        out.extend(sorted(sub.glob("phase2_*.txt")))
    return sorted(out)


def label_for(path: Path, base: Path, method: str) -> str:
    try:
        rel = path.parent.relative_to(base)
        folder = str(rel)
    except ValueError:
        folder = path.parent.name
    return f"{method}\n({folder})"


def pointcloud_colors(xyz: np.ndarray, rgb: np.ndarray) -> np.ndarray:
    """N x 3 float [0,1] for scatter."""
    if rgb.size and rgb.shape[0] == xyz.shape[0] and np.any(rgb):
        return (rgb.astype(np.float64) / 255.0).clip(0, 1)
    # height coloring
    z = xyz[:, 2]
    zmin, zmax = z.min(), z.max()
    if zmax - zmin < 1e-9:
        c = np.ones((len(xyz), 3)) * 0.7
        return c
    t = (z - zmin) / (zmax - zmin)
    cmap = plt.cm.viridis
    return cmap(t)[:, :3]


def plot_comparison(
    files: list[Path],
    base: Path,
    out_png: Path | None,
    tool_radius: float,
    raster_grid: int,
    cu_grid: int,
    recovery_json: Path | None,
) -> None:
    series: list[tuple[str, np.ndarray, Path]] = []
    clouds: list[tuple[np.ndarray, np.ndarray, Path | None]] = []
    waypoints_list: list[np.ndarray | None] = []
    for p in files:
        method, data = parse_phase2_txt(p)
        if data.size == 0:
            print(f"Warning: no data rows in {p}", file=sys.stderr)
            continue
        series.append((label_for(p, base, method), data, p))
        ply = find_target_ply(p)
        if ply and ply.is_file():
            try:
                xyz, rgb = load_ascii_ply(ply)
                clouds.append((xyz, rgb, ply))
            except Exception as e:
                print(f"Warning: could not load PLY {ply}: {e}", file=sys.stderr)
                clouds.append((np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8), None))
        else:
            clouds.append((np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8), None))

        wp_path = find_waypoints_txt(p)
        if wp_path and wp_path.is_file():
            try:
                wpa = load_waypoints_txt(wp_path)
                waypoints_list.append(wpa if wpa.shape[0] > 0 else None)
            except Exception as e:
                print(f"Warning: could not load waypoints {wp_path}: {e}", file=sys.stderr)
                waypoints_list.append(None)
        else:
            waypoints_list.append(None)

    if not series:
        print("No phase2_*.txt data to plot.", file=sys.stderr)
        raise SystemExit(2)

    # -------- metrics per run --------
    print("\n=== Metrics ===\n")
    for idx, ((label, data, pth), (c_xyz, c_rgb, ply_path)) in enumerate(zip(series, clouds)):
        print(f"--- {pth.name} ---")
        fr = data[:, 8:11]
        fd = data[:, 11:14]
        pos = data[:, 1:4]

        # Force RMS on PCA normal from cloud if available, else from positions only
        if c_xyz.shape[0] >= 3:
            c, n, _, _ = pca_plane_basis(c_xyz)
            # align normal with mean force direction for consistent sign
            mf = fr.mean(axis=0)
            if np.dot(n, mf) < 0:
                n = -n
            fn_r = fr @ n
            fn_d = fd @ n
            e_rms = float(np.sqrt(np.mean((fn_r - fn_d) ** 2)))
            print(f"  e_F^RMS (normal, PCA from target cloud) [N]: {e_rms:.6f}")
        else:
            n = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            fn_r = normal_force_series(fr, n)
            fn_d = normal_force_series(fd, n)
            e_rms = float(np.sqrt(np.mean((fn_r - fn_d) ** 2)))
            print(f"  e_F^RMS (Fz proxy, no PLY) [N]: {e_rms:.6f}")

        if c_xyz.shape[0] >= 3:
            c, _, u, v = pca_plane_basis(c_xyz)
            tgt_uv = project_uv(c_xyz, c, u, v)
            path_uv = project_uv(pos, c, u, v)
            hull = convex_hull_2d_mono(tgt_uv)
            area_t = polygon_area(hull)
            at_r, ac_in, ac_out, _ = raster_coverage_metrics(tgt_uv, path_uv, tool_radius, raster_grid)
            # paper-style proxies (no removal image)
            cr_proxy = 100.0 * ac_in / max(at_r, 1e-15)
            opr = 100.0 * ac_out / max(at_r, 1e-15)
            cu, _ = coverage_uniformity(path_uv, hull, cu_grid)
            print(f"  |Omega_target| (hull area) [m^2]: {area_t:.6f}")
            print(f"  CR_proxy = |cover ∩ target| / |target| [%]: {cr_proxy:.2f}  (path tube r={tool_radius*1000:.2f} mm)")
            print(f"  OPR = |cover \\ target| / |target| [%]: {opr:.2f}")
            print(f"  CU = std(n_k)/mean(n_k) [{cu_grid}x{cu_grid} grid]: {cu:.4f}")
        else:
            print("  CR, OPR, CU: n/a (no target PLY)")

        print()

    # Recovery metrics from optional JSON: list of { "t_disturb": float, "t_return": float, "success": bool }
    if recovery_json and recovery_json.is_file():
        try:
            ev = json.loads(recovery_json.read_text(encoding="utf-8"))
            if isinstance(ev, list) and ev:
                succ = sum(1 for e in ev if e.get("success", True))
                nd = len(ev)
                rsr = 100.0 * succ / max(nd, 1)
                times = []
                for e in ev:
                    td = float(e["t_disturb"])
                    tr = float(e["t_return"])
                    times.append(tr - td)
                print(f"RSR [%]: {rsr:.2f}  (N_dist={nd}, N_succ={succ})")
                print(f"T_rec mean [s]: {float(np.mean(times)):.4f}  (individual: {times})")
        except Exception as e:
            print(f"Warning: could not parse recovery JSON: {e}", file=sys.stderr)
    else:
        print("RSR, T_rec: n/a (pass --recovery-json; see script docstring)\n")

    # -------- figure 1: forces --------
    fig = plt.figure(figsize=(12, 10))
    fig.suptitle("Method comparison (phase 2 recordings)", fontsize=14)

    ax1 = fig.add_subplot(2, 2, 1)
    for label, data, _ in series:
        t = data[:, 0] - data[0, 0]
        fr = data[:, 8:11]
        ax1.plot(t, np.linalg.norm(fr, axis=1), label=label, alpha=0.85)
    ax1.set_xlabel("t - t0 [s]")
    ax1.set_ylabel("|F_real| [N]")
    ax1.set_title("Measured force magnitude")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=7, loc="best")

    ax2 = fig.add_subplot(2, 2, 2)
    for label, data, _ in series:
        t = data[:, 0] - data[0, 0]
        fd = data[:, 11:14]
        ax2.plot(t, np.linalg.norm(fd, axis=1), label=label, alpha=0.85)
    ax2.set_xlabel("t - t0 [s]")
    ax2.set_ylabel("|F_desired| [N]")
    ax2.set_title("Desired force magnitude")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=7, loc="best")

    ax3 = fig.add_subplot(2, 2, 3)
    for label, data, _ in series:
        t = data[:, 0] - data[0, 0]
        ax3.plot(t, data[:, 10], label=label, alpha=0.85)
    ax3.set_xlabel("t - t0 [s]")
    ax3.set_ylabel("Fz [N]")
    ax3.set_title("Real Fz (world)")
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=7, loc="best")

    ax4 = fig.add_subplot(2, 2, 4, projection="3d")
    for label, data, _ in series:
        ax4.plot(data[:, 1], data[:, 2], data[:, 3], label=label, alpha=0.85, linewidth=1.2)
    ax4.set_xlabel("px [m]")
    ax4.set_ylabel("py [m]")
    ax4.set_zlabel("pz [m]")
    ax4.set_title("Trajectories only")
    ax4.legend(fontsize=6, loc="upper left")

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    if out_png:
        fig.savefig(out_png.with_name(out_png.stem + "_forces" + out_png.suffix), dpi=150)
        print(f"Saved force figure to {out_png.with_name(out_png.stem + '_forces' + out_png.suffix)}")

    # -------- figure 2: point cloud + EE trajectory + recorded waypoint polyline --------
    fig2 = plt.figure(figsize=(11, 9))
    ax5 = fig2.add_subplot(111, projection="3d")
    max_pts = 8000
    first_cloud = True
    for si, ((label, data, pth), (c_xyz, c_rgb, ply_path), wp) in enumerate(
        zip(series, clouds, waypoints_list)
    ):
        color = f"C{si % 10}"
        if c_xyz.shape[0] > 0:
            step = max(1, len(c_xyz) // max_pts)
            xyz_s = c_xyz[::step]
            rgb_s = c_rgb[::step] if c_rgb.shape[0] == c_xyz.shape[0] else np.zeros((len(xyz_s), 3), dtype=np.uint8)
            cols = pointcloud_colors(xyz_s, rgb_s)
            ax5.scatter(
                xyz_s[:, 0],
                xyz_s[:, 1],
                xyz_s[:, 2],
                c=np.clip(cols, 0, 1),
                s=2,
                alpha=0.35,
                linewidths=0,
                label=f"PLY {ply_path.name}" if ply_path and first_cloud else None,
            )
            first_cloud = False
        ax5.plot(
            data[:, 1],
            data[:, 2],
            data[:, 3],
            color=color,
            label=label.split("\n")[0][:40],
            linewidth=2.0,
            alpha=0.95,
        )
        if wp is not None and wp.shape[0] >= 2:
            ax5.plot(
                wp[:, 0],
                wp[:, 1],
                wp[:, 2],
                color=color,
                linestyle="--",
                linewidth=2.4,
                alpha=0.95,
                label=f"waypoints ({pth.parent.name})",
            )
        elif wp is not None and wp.shape[0] == 1:
            ax5.scatter(
                [wp[0, 0]],
                [wp[0, 1]],
                [wp[0, 2]],
                color=color,
                marker="x",
                s=80,
                label=f"waypoint ({pth.parent.name})",
            )

    ax5.set_xlabel("x [m]")
    ax5.set_ylabel("y [m]")
    ax5.set_zlabel("z [m]")
    ax5.set_title("Target PLY + EE trajectory + recorded waypoint file (dashed)")
    ax5.legend(fontsize=7, loc="upper left")
    plt.tight_layout()

    if out_png:
        p2 = out_png.with_name(out_png.stem + "_cloud" + out_png.suffix)
        fig2.savefig(p2, dpi=150)
        print(f"Saved cloud figure to {p2}")
        plt.close(fig)
        plt.close(fig2)
    else:
        plt.show()


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(
        description="Plot phase2 recordings, overlay target PLY, compute coverage/force metrics."
    )
    p.add_argument("--data-dir", type=Path, default=script_dir, help="Base folder with run subdirs")
    p.add_argument("--runs", nargs="*", default=None, help="Only these timestamp subfolder names")
    p.add_argument("--output", type=Path, default=None, help="PNG base name (writes *_forces.png and *_cloud.png)")
    p.add_argument(
        "--tool-radius",
        type=float,
        default=0.004,
        help="Polishing tool radius [m] for coverage tube (default 4 mm)",
    )
    p.add_argument("--raster-grid", type=int, default=280, help="Raster resolution for CR/OPR")
    p.add_argument("--cu-grid", type=int, default=16, help="K = cu_grid^2 cells for uniformity (inside hull)")
    p.add_argument(
        "--recovery-json",
        type=Path,
        default=None,
        help='JSON list of events: [{"t_disturb":0,"t_return":1.2,"success":true}, ...]',
    )
    args = p.parse_args()
    base = args.data_dir.resolve()
    if not base.is_dir():
        print(f"Not a directory: {base}", file=sys.stderr)
        raise SystemExit(1)

    files = discover_phase2_files(base, args.runs if args.runs else None)
    if not files:
        print(f"No phase2_*.txt found under {base}", file=sys.stderr)
        raise SystemExit(2)

    print("Using files:")
    for f in files:
        print(f"  {f}")

    plot_comparison(
        files,
        base,
        args.output,
        args.tool_radius,
        args.raster_grid,
        args.cu_grid,
        args.recovery_json,
    )


if __name__ == "__main__":
    main()
