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
import sys

map_name = "Spielberg_map"
res = 1.0
origin = (0.0, 0.0)
mp = np.array([])

import numpy as np
from scipy import ndimage
from skimage.morphology import medial_axis

def generate_centerline_waypoints(map_array, threshold=254, spacing=10):
    """
    Generate waypoints along the middle of the corridor between two closed wall loops.

    Parameters
    ----------
    map_array : np.ndarray
        2D grayscale map. Free space should be 255, walls < threshold.
    threshold : int
        Pixels < threshold are considered walls.
    spacing : int
        Number of skeleton pixels to skip between output waypoints.

    Returns
    -------
    np.ndarray
        Array of shape (N, 2), each row is [x, y]
    """

    # True = wall
    wall_mask = map_array < threshold

    # Label connected wall components
    wall_labels, num_walls = ndimage.label(wall_mask)

    if num_walls < 2:
        return np.empty((0, 2), dtype=np.int32)

    # For each wall component, compute the area of its filled interior
    wall_infos = []
    for label_id in range(1, num_walls + 1):
        component = (wall_labels == label_id)

        # Fill the inside of this closed wall loop
        filled = ndimage.binary_fill_holes(component)
        filled_area = np.sum(filled)

        wall_infos.append({
            "label": label_id,
            "component": component,
            "filled": filled,
            "filled_area": filled_area
        })

    # Sort by filled area descending
    wall_infos.sort(key=lambda x: x["filled_area"], reverse=True)

    # Outer wall is usually the largest closed wall
    outer = wall_infos[0]

    # Inner wall should be the next largest closed wall that lies inside the outer one
    inner = None
    for candidate in wall_infos[1:]:
        # Check whether candidate lies inside outer filled region
        candidate_pixels = np.argwhere(candidate["component"])
        if len(candidate_pixels) == 0:
            continue

        y0, x0 = candidate_pixels[0]
        if outer["filled"][y0, x0]:
            inner = candidate
            break

    if inner is None:
        return np.empty((0, 2), dtype=np.int32)

    # Corridor = inside outer wall, but outside inner wall, and not on wall pixels
    corridor_mask = outer["filled"] & (~inner["filled"]) & (~wall_mask)

    if not np.any(corridor_mask):
        return np.empty((0, 2), dtype=np.int32)

    # Skeletonize only the corridor
    skeleton, _ = medial_axis(corridor_mask, return_distance=True)

    ys, xs = np.where(skeleton)
    points = np.column_stack((xs, ys))

    if len(points) == 0:
        return np.empty((0, 2), dtype=np.int32)

    # Build neighbor graph using 8-connectivity
    point_set = set(map(tuple, points))
    neighbors = {}

    for x, y in points:
        nbrs = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                candidate = (x + dx, y + dy)
                if candidate in point_set:
                    nbrs.append(candidate)
        neighbors[(x, y)] = nbrs

    # Prefer degree-2 points for loop traversal
    loop_points = [p for p, nbrs in neighbors.items() if len(nbrs) == 2]
    if not loop_points:
        loop_points = list(neighbors.keys())

    loop_points_arr = np.array(loop_points, dtype=float)

    # Start from the rightmost point on the track corridor
    start_idx = np.argmax(loop_points_arr[:, 0])
    start = tuple(loop_points_arr[start_idx].astype(int))

    # Walk through skeleton in order
    ordered = [start]
    visited = {start}
    prev = None
    current = start

    while True:
        nbrs = neighbors[current]
        candidates = [p for p in nbrs if p != prev and p not in visited]

        if len(candidates) == 0:
            break

        if prev is not None and len(candidates) > 1:
            vx = current[0] - prev[0]
            vy = current[1] - prev[1]

            def score(p):
                wx = p[0] - current[0]
                wy = p[1] - current[1]
                return vx * wx + vy * wy

            next_pt = max(candidates, key=score)
        else:
            next_pt = candidates[0]

        ordered.append(next_pt)
        visited.add(next_pt)
        prev = current
        current = next_pt

        if len(ordered) > 10 and start in neighbors[current]:
            break

    ordered = np.array(ordered, dtype=np.int32)

    if len(ordered) == 0:
        return np.empty((0, 2), dtype=np.int32)

    # Downsample
    waypoints = ordered[::spacing]

    return waypoints

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

def main():
    if len(sys.argv) == 2:
        map_name = sys.argv[1]
    print(map_name)
    #----
    # Import Map Image
    map_img = Image.open(map_name + ".png")
    global mp, res, origin
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
    mpt = thicken_walls(mp)
    wp_2d = generate_centerline_waypoints(mpt, threshold = 250, spacing = 50)
    wp_list = [pixel_to_map(i[0], i[1]) for i in wp_2d]
    wp_list = add_heading_to_waypoints(wp_list)


    #---- 
    # Process waypoints
    GAP = 1.0
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

    for i in range(len(wp_list) - 1):
        heading = wp_list[i+1][2] + np.pi
        g_vec = (np.sin(heading), np.cos(heading))
        pt_list.extend(ptsp.generate_brachistochrone(wp_list[i+1][:2], wp_list[i][:2], g_vec=g_vec)[::-1])

    heading = wp_list[0][2] + np.pi
    g_vec = (np.sin(heading), np.cos(heading))
    pt_list.extend(ptsp.generate_brachistochrone(wp_list[0][:2], wp_list[-1][:2], g_vec=g_vec)[::-1])

    pt_list = filter_waypoints_by_distance(pt_list, GAP)


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
    # plt.show()
    plt.savefig("waypoints_preview", dpi=500)

    #pt_list = [pixel_to_map(x, y) for x,y in pt_list]

    #----
    # Save to file
    with open(map_name + '_br_waypoints.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(pt_list)  # Use writerow(list) for a single row

if __name__ == "__main__":
    main()