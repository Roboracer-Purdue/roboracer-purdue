#!/usr/bin/env python3
"""
Generate F1TENTH-style waypoint CSV from a map image + map YAML.

Output columns:
    x, y, yaw, curvature, velocity

Main idea:
1. Read occupancy-map image.
2. Extract only the enclosed track corridor, not the outside white background.
3. Skeletonize the corridor.
4. Build an 8-connected skeleton graph and trace the longest loop/path.
5. Convert pixels to world coordinates using map resolution and origin.
6. Smooth, compute yaw, curvature, and curvature-based velocity.

Usage:
    python3 generate_waypoints_from_map.py --yaml IV_2026_SIM_clean.yaml

Optional:
    python3 generate_waypoints_from_map.py --yaml IV_2026_SIM_clean.yaml --out generated_waypoints.csv --show
"""

import argparse
from pathlib import Path

import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image

from scipy.ndimage import binary_fill_holes, binary_erosion, binary_dilation
from scipy.signal import savgol_filter
from skimage.measure import label, regionprops
from skimage.morphology import skeletonize, remove_small_objects


# -----------------------------
# Utility functions
# -----------------------------

def wrap_to_pi(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def pixel_to_world(xs, ys, resolution, origin_x, origin_y, image_height):
    """Convert image pixel coordinates to ROS map/world coordinates."""
    x_world = xs * resolution + origin_x
    y_world = (image_height - ys) * resolution + origin_y
    return x_world, y_world


def world_to_pixel(x, y, resolution, origin_x, origin_y, image_height):
    """Convert ROS map/world coordinates to image pixel coordinates."""
    x_pix = (x - origin_x) / resolution
    y_pix = image_height - (y - origin_y) / resolution
    return x_pix, y_pix


def extract_track_corridor(arr, wall_threshold=100, free_threshold=200, erosion_iters=2):
    """
    Extract the enclosed drivable track corridor.

    For maps like yours, the outside background is also white, so using arr > 240
    directly makes the algorithm think the entire outside world is drivable.

    This function instead:
    - treats dark pixels as walls,
    - fills enclosed regions,
    - keeps the largest filled component,
    - intersects with white/free pixels.
    """
    walls = arr < wall_threshold

    # Slightly close tiny wall gaps before fill. This helps if the border line is thin.
    walls_closed = binary_dilation(walls, iterations=1)
    filled = binary_fill_holes(walls_closed)

    labels = label(filled)
    regions = regionprops(labels)
    if not regions:
        raise RuntimeError("Could not find any enclosed track region. Try changing wall_threshold.")

    largest = max(regions, key=lambda r: r.area)
    track_region = labels == largest.label

    # Free corridor = enclosed region minus wall pixels.
    free = track_region & (arr > free_threshold)

    # Remove tiny junk, then erode to stay away from walls/islands.
    free = remove_small_objects(free, min_size=100)
    if erosion_iters > 0:
        free = binary_erosion(free, iterations=erosion_iters)

    return free, track_region


def build_skeleton_graph(skel):
    """
    Build an 8-connected graph from skeleton pixels.
    Nodes are (x, y) tuples.
    """
    ys, xs = np.where(skel)
    nodes = set(zip(xs.tolist(), ys.tolist()))
    graph = {p: [] for p in nodes}

    nbr_offsets = [
        (-1, -1), (0, -1), (1, -1),
        (-1,  0),          (1,  0),
        (-1,  1), (0,  1), (1,  1),
    ]

    for x, y in nodes:
        for dx, dy in nbr_offsets:
            q = (x + dx, y + dy)
            if q in nodes:
                graph[(x, y)].append(q)

    return graph


def connected_components_graph(graph):
    seen = set()
    comps = []
    for node in graph:
        if node in seen:
            continue
        stack = [node]
        seen.add(node)
        comp = []
        while stack:
            u = stack.pop()
            comp.append(u)
            for v in graph[u]:
                if v not in seen:
                    seen.add(v)
                    stack.append(v)
        comps.append(comp)
    return comps


def keep_largest_graph_component(graph):
    comps = connected_components_graph(graph)
    if not comps:
        raise RuntimeError("Skeleton graph has no connected components.")
    largest = max(comps, key=len)
    keep = set(largest)
    return {p: [q for q in graph[p] if q in keep] for p in largest}


def trace_skeleton_path(graph, start_hint=None):
    """
    Trace path by walking the graph, avoiding nearest-neighbor jumps.

    This works best when the largest skeleton component is mostly a single loop/path.
    For imperfect skeletons, it still avoids teleporting through islands because it only
    follows actual neighboring skeleton pixels.
    """
    graph = keep_largest_graph_component(graph)

    nodes = list(graph.keys())
    degrees = {p: len(graph[p]) for p in nodes}
    endpoints = [p for p, d in degrees.items() if d == 1]

    if start_hint is not None:
        sx, sy = start_hint
        start = min(nodes, key=lambda p: (p[0] - sx) ** 2 + (p[1] - sy) ** 2)
    elif endpoints:
        # If it is an open path, start from an endpoint.
        start = min(endpoints, key=lambda p: p[1])
    else:
        # If it is a loop, choose top-left-ish node.
        start = min(nodes, key=lambda p: (p[1], p[0]))

    path = []
    visited_edges = set()
    prev = None
    curr = start

    def edge_key(a, b):
        return tuple(sorted([a, b]))

    max_steps = len(nodes) * 4
    for _ in range(max_steps):
        path.append(curr)

        candidates = []
        for n in graph[curr]:
            if n == prev:
                continue
            if edge_key(curr, n) in visited_edges:
                continue
            candidates.append(n)

        if not candidates:
            # allow returning to previous only if truly stuck
            break

        # Prefer continuing straight instead of taking branch into island/spur.
        if prev is not None and len(candidates) > 1:
            vx = curr[0] - prev[0]
            vy = curr[1] - prev[1]
            candidates.sort(key=lambda n: -((n[0] - curr[0]) * vx + (n[1] - curr[1]) * vy))

        nxt = candidates[0]
        visited_edges.add(edge_key(curr, nxt))
        prev, curr = curr, nxt

        # Closed loop completed.
        if curr == start and len(path) > 10:
            break

    return np.array(path, dtype=float)


def resample_closed_path(x, y, spacing=0.20, closed=True):
    """Resample path to roughly uniform spacing in meters."""
    pts = np.column_stack([x, y])
    if closed:
        pts2 = np.vstack([pts, pts[0]])
    else:
        pts2 = pts

    seg = np.linalg.norm(np.diff(pts2, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]

    if total < spacing * 3:
        return x, y

    new_s = np.arange(0.0, total, spacing)
    new_x = np.interp(new_s, s, pts2[:, 0])
    new_y = np.interp(new_s, s, pts2[:, 1])
    return new_x, new_y


def smooth_path(x, y, window=31, polyorder=3):
    n = len(x)
    if n < 9:
        return x, y

    # Window must be odd and smaller than n.
    win = min(window, n - 1 if (n - 1) % 2 == 1 else n - 2)
    if win < 7:
        return x, y

    x_s = savgol_filter(x, win, polyorder, mode="wrap")
    y_s = savgol_filter(y, win, polyorder, mode="wrap")
    return x_s, y_s


def compute_yaw_curvature_velocity(x, y, max_speed=3.0, min_speed=1.0, curvature_gain=3.0):
    dx = np.gradient(x)
    dy = np.gradient(y)

    yaw = np.unwrap(np.arctan2(dy, dx))

    ds = np.sqrt(dx ** 2 + dy ** 2)
    ds[ds < 1e-6] = 1e-6

    dyaw = np.gradient(yaw)
    curvature = dyaw / ds

    # Smooth curvature a bit if possible.
    n = len(curvature)
    win = min(31, n - 1 if (n - 1) % 2 == 1 else n - 2)
    if win >= 7:
        curvature = savgol_filter(curvature, win, 3, mode="wrap")

    velocity = max_speed / (1.0 + curvature_gain * np.abs(curvature))
    velocity = np.clip(velocity, min_speed, max_speed)

    return wrap_to_pi(yaw), curvature, velocity


# -----------------------------
# Main
# -----------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml", required=True, help="Map YAML file, e.g. IV_2026_SIM_clean.yaml")
    parser.add_argument("--out", default="generated_waypoints.csv", help="Output CSV path")
    parser.add_argument("--spacing", type=float, default=0.20, help="Waypoint spacing in meters")
    parser.add_argument("--max-speed", type=float, default=3.0)
    parser.add_argument("--min-speed", type=float, default=1.0)
    parser.add_argument("--curvature-gain", type=float, default=3.0)
    parser.add_argument("--wall-threshold", type=int, default=100)
    parser.add_argument("--free-threshold", type=int, default=200)
    parser.add_argument("--erosion-iters", type=int, default=2)
    parser.add_argument("--start-x", type=float, default=None, help="Optional start x in world coordinates")
    parser.add_argument("--start-y", type=float, default=None, help="Optional start y in world coordinates")
    parser.add_argument("--show", action="store_true", help="Show debug plots")
    args = parser.parse_args()

    yaml_path = Path(args.yaml)
    with open(yaml_path, "r") as f:
        info = yaml.safe_load(f)

    img_path = Path(info["image"])
    if not img_path.is_absolute():
        img_path = yaml_path.parent / img_path

    resolution = float(info["resolution"])
    origin_x, origin_y, _ = info["origin"]

    img = Image.open(img_path).convert("L")
    arr = np.array(img)
    h, w = arr.shape

    free, track_region = extract_track_corridor(
        arr,
        wall_threshold=args.wall_threshold,
        free_threshold=args.free_threshold,
        erosion_iters=args.erosion_iters,
    )

    skel = skeletonize(free)
    graph = build_skeleton_graph(skel)

    start_hint = None
    if args.start_x is not None and args.start_y is not None:
        sx, sy = world_to_pixel(args.start_x, args.start_y, resolution, origin_x, origin_y, h)
        start_hint = (sx, sy)

    path_pix = trace_skeleton_path(graph, start_hint=start_hint)
    if len(path_pix) < 10:
        raise RuntimeError("Traced path is too short. Try erosion_iters=0 or adjust thresholds.")

    xs_pix = path_pix[:, 0]
    ys_pix = path_pix[:, 1]

    x, y = pixel_to_world(xs_pix, ys_pix, resolution, origin_x, origin_y, h)

    x, y = resample_closed_path(x, y, spacing=args.spacing, closed=False)
    x, y = smooth_path(x, y, window=31, polyorder=3)

    yaw, curvature, velocity = compute_yaw_curvature_velocity(
        x,
        y,
        max_speed=args.max_speed,
        min_speed=args.min_speed,
        curvature_gain=args.curvature_gain,
    )

    df = pd.DataFrame({
        "x": x,
        "y": y,
        "yaw": yaw,
        "curvature": curvature,
        "velocity": velocity,
    })

    out_path = Path(args.out)
    df.to_csv(out_path, index=False)
    print(f"Saved {out_path} with {len(df)} waypoints")

    if args.show:
        # Plot waypoint path over map.
        xpix_plot, ypix_plot = world_to_pixel(x, y, resolution, origin_x, origin_y, h)

        plt.figure(figsize=(10, 8))
        plt.imshow(img, cmap="gray", origin="upper")
        plt.plot(xpix_plot, ypix_plot, linewidth=1.5, label="generated waypoints")
        plt.scatter(xpix_plot[0], ypix_plot[0], s=80, marker="o", label="start")
        plt.axis("equal")
        plt.legend()
        plt.title("Generated waypoints over map")

        plt.figure()
        plt.plot(velocity)
        plt.title("Velocity profile")
        plt.xlabel("Waypoint index")
        plt.ylabel("Velocity")

        plt.figure()
        plt.plot(curvature)
        plt.title("Curvature profile")
        plt.xlabel("Waypoint index")
        plt.ylabel("Curvature")

        plt.show()


if __name__ == "__main__":
    main()
