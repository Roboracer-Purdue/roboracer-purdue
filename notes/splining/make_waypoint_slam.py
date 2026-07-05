from PIL import Image
import numpy as np 
import math
import matplotlib.pyplot as plt 
import yaml
from scipy.interpolate import splprep, splev
from scipy.ndimage import binary_dilation
from scipy.spatial import cKDTree
from scipy import ndimage
from skimage.morphology import medial_axis
import points_test as ptsp
import csv

map_name = "IV_2026_SIM_clean"
res = 1.0
origin = (4.5, 5.1)

import numpy as np
from scipy import ndimage
from skimage.morphology import medial_axis
import numpy as np
from scipy import ndimage
from skimage.morphology import skeletonize
from collections import deque


import numpy as np
from scipy import ndimage
from skimage.morphology import skeletonize
from collections import deque

def remove_small_obstacles(map_array, obstacle_threshold=100, min_size=200):
    """
    Remove tiny obstacle blobs such as table legs from a grayscale occupancy map.

    Parameters
    ----------
    map_array : np.ndarray
        2D grayscale map.
    obstacle_threshold : int
        Pixels < obstacle_threshold are treated as occupied.
    min_size : int
        Connected occupied components smaller than this are removed.

    Returns
    -------
    np.ndarray
        Cleaned map array.
    """
    cleaned = map_array.copy()

    # True = occupied
    obstacle_mask = map_array < obstacle_threshold

    labels, num = ndimage.label(obstacle_mask)
    if num == 0:
        return cleaned

    counts = np.bincount(labels.ravel())

    for label_id in range(1, num + 1):
        if counts[label_id] < min_size:
            cleaned[labels == label_id] = 254  # turn tiny obstacle into free space

    return cleaned

def generate_centerline_waypoints(
    map_array,
    spacing=10,
    free_threshold=250,
    start_point=None
):
    """
    Generate ordered centerline waypoints for a single-loop corridor map.

    Parameters
    ----------
    map_array : np.ndarray
        2D grayscale map.
    spacing : int
        Take every `spacing`-th point from the ordered centerline.
    free_threshold : int
        Pixels > free_threshold are treated as free space.
    start_point : tuple[int, int] or None
        Pixel coordinate (x, y) near where you want the loop to begin.

    Returns
    -------
    np.ndarray
        Array of shape (N, 2), each row is [x, y].
    """

    # 1) Free-space mask
    free_mask = map_array > free_threshold

    # Light cleanup
    free_mask = ndimage.binary_opening(free_mask, iterations=1)
    free_mask = ndimage.binary_closing(free_mask, iterations=1)

    if not np.any(free_mask):
        return np.empty((0, 2), dtype=np.int32)

    # 2) Connected components in free space
    labels, num_labels = ndimage.label(free_mask)
    if num_labels == 0:
        return np.empty((0, 2), dtype=np.int32)

    # Choose region nearest start_point, otherwise largest
    if start_point is not None:
        sx, sy = start_point
        best_label = None
        best_dist = np.inf

        for label_id in range(1, num_labels + 1):
            ys, xs = np.where(labels == label_id)
            if len(xs) == 0:
                continue

            d2 = (xs - sx) ** 2 + (ys - sy) ** 2
            min_d2 = np.min(d2)

            if min_d2 < best_dist:
                best_dist = min_d2
                best_label = label_id
    else:
        counts = np.bincount(labels.ravel())
        counts[0] = 0
        best_label = np.argmax(counts)

    if best_label is None:
        return np.empty((0, 2), dtype=np.int32)

    track_region = (labels == best_label)

    if not np.any(track_region):
        return np.empty((0, 2), dtype=np.int32)

    # 3) Skeletonize
    skeleton = skeletonize(track_region)

    ys, xs = np.where(skeleton)
    points = [(int(x), int(y)) for x, y in zip(xs, ys)]

    if not points:
        return np.empty((0, 2), dtype=np.int32)

    # 4) Build 8-connected graph
    point_set = set(points)
    neighbors = {p: [] for p in points}

    for x, y in points:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                q = (x + dx, y + dy)
                if q in point_set:
                    neighbors[(x, y)].append(q)

    # 5) Prune leaf branches iteratively
    deg = {p: len(neighbors[p]) for p in points}
    queue = deque([p for p, d in deg.items() if d <= 1])
    removed = set()

    while queue:
        p = queue.popleft()
        if p in removed:
            continue

        removed.add(p)

        for q in neighbors[p]:
            if q not in removed:
                deg[q] -= 1
                if deg[q] == 1:
                    queue.append(q)

    core_points = [p for p in points if p not in removed]
    if not core_points:
        return np.empty((0, 2), dtype=np.int32)

    core_set = set(core_points)
    core_neighbors = {
        p: [q for q in neighbors[p] if q in core_set]
        for p in core_points
    }

    # 6) Keep largest remaining connected component
    remaining = set(core_points)
    components = []

    while remaining:
        seed = next(iter(remaining))
        comp = []
        q = deque([seed])
        seen = {seed}

        while q:
            u = q.popleft()
            comp.append(u)
            for v in core_neighbors[u]:
                if v not in seen:
                    seen.add(v)
                    q.append(v)

        components.append(comp)
        remaining -= set(comp)

    loop = max(components, key=len)
    loop_set = set(loop)

    loop_neighbors = {
        p: [q for q in core_neighbors[p] if q in loop_set]
        for p in loop
    }

    loop_arr = np.array(loop, dtype=np.int32)
    if len(loop_arr) == 0:
        return np.empty((0, 2), dtype=np.int32)

    # 7) Pick starting node nearest requested start_point
    if start_point is not None:
        sx, sy = start_point
        d2 = (loop_arr[:, 0] - sx) ** 2 + (loop_arr[:, 1] - sy) ** 2
        start = tuple(loop_arr[np.argmin(d2)])
    else:
        start = tuple(loop_arr[0])

    # 8) Walk the loop in graph order
    ordered = [start]
    prev = None
    current = start
    visited = {start}

    while True:
        nbrs = loop_neighbors[current]

        # Prefer not to go back where we came from
        candidates = [n for n in nbrs if n != prev]

        if not candidates:
            break

        # In a clean loop, there should be exactly one forward candidate
        next_node = None
        for c in candidates:
            if c not in visited:
                next_node = c
                break

        # If both neighbors already visited, we likely closed the loop
        if next_node is None:
            break

        ordered.append(next_node)
        visited.add(next_node)
        prev = current
        current = next_node

        if current == start:
            break

    ordered = np.array(ordered, dtype=np.int32)

    if len(ordered) == 0:
        return np.empty((0, 2), dtype=np.int32)

    # 9) Optionally reverse direction if the opposite direction is closer
    if start_point is not None and len(ordered) > 2:
        sx, sy = start_point

        forward_next = ordered[min(1, len(ordered) - 1)]
        reverse_next = ordered[-1]

        df = (forward_next[0] - sx) ** 2 + (forward_next[1] - sy) ** 2
        dr = (reverse_next[0] - sx) ** 2 + (reverse_next[1] - sy) ** 2

        if dr < df:
            ordered = np.concatenate(([ordered[0]], ordered[:0:-1]), axis=0)

    # 10) Downsample
    return ordered[::spacing]

def add_heading_to_waypoints(waypoints, closed_loop=True):
    """
    Add heading angle to each waypoint.

    Parameters
    ----------
    waypoints : np.ndarray
        Array of shape (N, 2), each row is [x, y]
    closed_loop : bool
        If True, heading of the last point is computed toward the first point.

    Returns
    -------
    np.ndarray
        Array of shape (N, 3), each row is [x, y, heading]
        where heading is in radians.
    """
    if len(waypoints) == 0:
        return np.empty((0, 3), dtype=float)

    if len(waypoints) == 1:
        x, y = waypoints[0]
        return np.array([[x, y, 0.0]], dtype=float)

    result = []

    n = len(waypoints)

    for i in range(n):
        x, y = waypoints[i]

        if i < n - 1:
            next_x, next_y = waypoints[i + 1]
        else:
            if closed_loop:
                next_x, next_y = waypoints[0]
            else:
                next_x, next_y = waypoints[i]

        dx = next_x - x
        dy = next_y - y
        heading = np.arctan2(dx, dy)

        result.append([x, y, heading])

    return np.array(result, dtype=float)

def thicken_walls(map_array, threshold=254, iterations=2):
    """
    Increase wall thickness and convert map to binary (0 or 255)

    Parameters:
        map_array (np.ndarray): 2D grayscale map (0–255)
        threshold (int): values < threshold are considered walls
        iterations (int): how much to thicken the walls

    Returns:
        np.ndarray: processed map with only 0 (wall) and 255 (free)
    """

    # Step 1: Identify walls (True = wall)
    wall_mask = map_array < threshold

    # Step 2: Dilate walls to increase thickness
    thick_walls = binary_dilation(wall_mask, iterations=iterations)

    # Step 3: Convert back to 0 (wall) and 255 (free space)
    result = np.where(thick_walls, 0, 255).astype(np.uint8)

    return result

def map_to_pixel(ox, oy):
    ox -= origin[0]
    oy -= origin[1]
    
    ox /= res
    oy /= res

    oy = mp.shape[1] - oy
    # ox = mp.shape[0] - ox

    return ox, oy

def pixel_to_map(ox, oy):
    oy = mp.shape[1] - oy

    ox *= res
    oy *= res

    ox += origin[0]
    oy += origin[1]

    return ox, oy

def filter_waypoints_by_distance(waypoints, min_dist=0.1):
    """
    Remove waypoints that are too close to each other.

    Parameters
    ----------
    waypoints : list or np.ndarray
        [(x,y) or (x,y,...) ...]
    min_dist : float
        Minimum distance between consecutive kept points

    Returns
    -------
    np.ndarray
    """
    if len(waypoints) == 0:
        return np.array([])

    filtered = [np.array(waypoints[0][:2], dtype=float)]
    last = filtered[0]

    for wp in waypoints[1:]:
        p = np.array(wp[:2], dtype=float)
        if np.linalg.norm(p - last) >= min_dist:
            filtered.append(p)
            last = p

    return np.array(filtered)

#----
# Import Map Image
map_img = Image.open(map_name + ".png")
mp = np.array(map_img)

#----
# Read map yaml
with open(map_name + ".yaml", "r") as f:
    map_config = yaml.safe_load(f)

# print(map_config)
res = map_config["resolution"]
origin = map_config["origin"][0], map_config["origin"][1] 

#----
# Create waypoints from map

clean_map = remove_small_obstacles(
    mp,
    obstacle_threshold=200,
    min_size=5
)

mpt = thicken_walls(clean_map, threshold = 120, iterations = 2)
plt.imshow(mpt)
plt.show()

wp_2d = generate_centerline_waypoints(mpt, spacing = 30)
wp_list = [pixel_to_map(i[0], i[1]) for i in wp_2d]
wp_list = add_heading_to_waypoints(wp_list)



#---- 
# Process waypoints
GAP = 0.6
CONT_DIST = 1.0
STEP_BACK = 0.0

'''
wp_list = [
    (-13.6974, 7.42467 - STEP_BACK, np.pi),
    (-13.7382, 0.808186 + STEP_BACK, np.pi),
    (-12.9346 + STEP_BACK, 0.0593758, np.pi / 2),
    (8.41934 - STEP_BACK, -0.252186, np.pi / 2),
    (9.66924, 0.827478 + STEP_BACK, 0),
    (9.58695, 7.8315 - STEP_BACK, 0),
    (8.76503 - STEP_BACK, 8.54678, -np.pi / 2),
    (-12.8128 + STEP_BACK, 8.65672, -np.pi / 2)
]
'''

pt_list = []

# Bezier
'''
for i in range(len(wp_list) - 1):
    num_points=int(np.floor(math.dist(wp_list[i+1], wp_list[i]) / GAP))
    pt_list.extend(ptsp.generate_bezier_curve(wp_list[i], wp_list[i+1], num_points=num_points, control_dist=CONT_DIST))

num_points=int(np.floor(math.dist(wp_list[-1], wp_list[0]) / GAP))
pt_list.extend(ptsp.generate_bezier_curve(wp_list[-1], wp_list[0], num_points=num_points, control_dist=CONT_DIST))
'''
# Bachistochrone
'''
for i in range(len(wp_list) - 1):
    heading = wp_list[i+1][2] + np.pi
    g_vec = (np.sin(heading), np.cos(heading))
    pt_list.extend(ptsp.generate_brachistochrone(wp_list[i+1][:2], wp_list[i][:2], g_vec=g_vec)[::-1])

heading = wp_list[0][2] + np.pi
g_vec = (np.sin(heading), np.cos(heading))
pt_list.extend(ptsp.generate_brachistochrone(wp_list[0][:2], wp_list[-1][:2], g_vec=g_vec)[::-1])

pt_list = filter_waypoints_by_distance(pt_list, GAP)
'''
px_list = []
py_list = []

for i in wp_list:
    x,y = map_to_pixel(i[0],i[1])
    i = (x,y,i[2])
    ptsp.plot_vector(i, 0.2)

for x, y in pt_list:
    pixel_point = map_to_pixel(x,y)

    px_list.append(pixel_point[0])
    py_list.append(pixel_point[1])

#----
# Map is usually a gray scale image
plt.imshow(mp)
plt.plot(px_list, py_list,"b.")
# plt.xlim(700, 1400) # levine
# plt.ylim(700, 1200) # levine
plt.show()
plt.savefig("waypoints_preview", dpi=500)

#----
# Save to file
with open('spiel_waypoints_br.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(pt_list)  # Use writerow(list) for a single row