from PIL import Image
import numpy as np 
import math
import matplotlib.pyplot as plt 
import yaml
from scipy.interpolate import splprep, splev
import points_test as ptsp
import csv

map_name = "levine"
res = 1.0
origin = (0.0, 0.0)

# ChatGPT
def spline_waypoints(goals, num_waypoints=6, smoothness=0.0, degree=3):
    # Read x,y columns
    df = goals
    x, y = zip(*goals)
    
    # Optional: remove consecutive duplicate points
    pts = np.column_stack((x, y))
    keep = np.ones(len(pts), dtype=bool)
    keep[1:] = np.any(np.diff(pts, axis=0) != 0, axis=1)
    pts = pts[keep]

    # Spline degree k must be < number of points
    k = min(degree, len(pts) - 1)

    # Fit parametric spline x(u), y(u)
    # s=0 -> interpolate through every point exactly
    # s>0 -> smoother curve that may not hit every point exactly
    tck, u = splprep([pts[:, 0], pts[:, 1]], s=smoothness, k=k)

    # Sample evenly in spline parameter space
    u_new = np.linspace(0, 1, num_waypoints)
    x_new, y_new = splev(u_new, tck)

    # Return as (N,2) array
    return np.column_stack((x_new, y_new))

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
map_img = Image.open("levine" + ".png")
mp = np.array(map_img)

#----
# Read map yaml
with open("levine" + ".yaml", "r") as f:
    map_config = yaml.safe_load(f)

# print(map_config)
res = map_config["resolution"]
origin = map_config["origin"][0], map_config["origin"][1] 

#---- 
# Process waypoints
GAP = 0.6
CONT_DIST = 1.0
STEP_BACK = 0.0

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

pt_list = []

'''
for i in range(len(wp_list) - 1):
    num_points=int(np.floor(math.dist(wp_list[i+1], wp_list[i]) / GAP))
    pt_list.extend(ptsp.generate_bezier_curve(wp_list[i], wp_list[i+1], num_points=num_points, control_dist=CONT_DIST))

num_points=int(np.floor(math.dist(wp_list[-1], wp_list[0]) / GAP))
pt_list.extend(ptsp.generate_bezier_curve(wp_list[-1], wp_list[0], num_points=num_points, control_dist=CONT_DIST))
'''
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
    ptsp.plot_vector(i)

for x, y in pt_list:
    pixel_point = map_to_pixel(x,y)

    px_list.append(pixel_point[0])
    py_list.append(pixel_point[1])

#----
# Map is usually a gray scale image
plt.imshow(mp)
plt.plot(px_list, py_list,"b.")
plt.xlim(700, 1400) # levine
plt.ylim(700, 1200) # levine
plt.savefig("waypoints_preview", dpi=500)

#----
# Save to file
with open('waypoints_baris_reverse.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(pt_list)  # Use writerow(list) for a single row