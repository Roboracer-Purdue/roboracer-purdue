import numpy as np
import matplotlib.pyplot as plt

def generate_bezier_curve(start, goal, num_points=50, control_dist=1.0):
    """
    Generate points on a cubic Bezier curve between two poses.

    Parameters
    ----------
    start : tuple
        (x, y, theta) start pose
    goal : tuple
        (x, y, theta) goal pose
    num_points : int
        Number of sampled points on the curve
    control_dist : float
        Distance of control points from start/goal along heading directions

    Returns
    -------
    np.ndarray
        Array of shape (num_points, 2) containing x,y points on the curve
    """
    x0, y0, th0 = start
    x3, y3, th3 = goal

    B0 = np.array([x0, y0], dtype=float)
    B3 = np.array([x3, y3], dtype=float)

    # Heading directions in your coordinate system
    dir0 = np.array([np.sin(th0), np.cos(th0)], dtype=float)
    dir3 = np.array([np.sin(th3), np.cos(th3)], dtype=float)

    # Control points
    B1 = B0 + control_dist * dir0
    B2 = B3 - control_dist * dir3

    t_values = np.linspace(0.0, 1.0, num_points)
    points = []

    for t in t_values:
        p = ((1 - t)**3 * B0
             + 3 * (1 - t)**2 * t * B1
             + 3 * (1 - t) * t**2 * B2
             + t**3 * B3)
        points.append(p)

    return np.array(points)

def generate_hermite_curve(start, goal, num_points=50, scale=1.0):
    x0, y0, th0 = start
    x1, y1, th1 = goal

    P0 = np.array([x0, y0])
    P1 = np.array([x1, y1])

    # Convert heading to direction vectors (your coordinate system)
    T0 = scale * np.array([np.sin(th0), np.cos(th0)])
    T1 = scale * np.array([np.sin(th1), np.cos(th1)])

    points = []

    for t in np.linspace(0, 1, num_points):
        h00 = 2*t**3 - 3*t**2 + 1
        h10 = t**3 - 2*t**2 + t
        h01 = -2*t**3 + 3*t**2
        h11 = t**3 - t**2

        p = h00 * P0 + h10 * T0 + h01 * P1 + h11 * T1
        points.append(p)

    return np.array(points)

def generate_brachistochrone(start_xy, goal_xy, g_vec=(0.0, -1.0),
                             num_points=100, tol=1e-10, max_iter=200):
    """
    Generate a brachistochrone curve between two points for a given gravity direction.

    Parameters
    ----------
    start_xy : tuple
        (x0, y0)
    goal_xy : tuple
        (x1, y1)
    g_vec : tuple
        Gravity direction vector. Does not need to be unit length.
    num_points : int
        Number of sampled points
    tol : float
        Numerical tolerance
    max_iter : int
        Maximum bisection iterations

    Returns
    -------
    np.ndarray
        Shape (num_points, 2)
    """
    P0 = np.array(start_xy, dtype=float)
    P1 = np.array(goal_xy, dtype=float)

    g = np.array(g_vec, dtype=float)
    g_norm = np.linalg.norm(g)
    if g_norm == 0:
        raise ValueError("g_vec must be non-zero.")
    g_hat = g / g_norm

    d = P1 - P0
    drop = np.dot(d, g_hat)
    if drop <= 0:
        raise ValueError("Goal must lie downhill in the gravity direction.")

    # Pick a perpendicular direction
    e_hat = np.array([-g_hat[1], g_hat[0]])

    # Flip it if needed so the goal lies in the positive local x direction
    if np.dot(d, e_hat) < 0:
        e_hat = -e_hat

    horiz = np.dot(d, e_hat)

    # Special case: nearly vertical descent
    if abs(horiz) < tol:
        t = np.linspace(0.0, 1.0, num_points)
        points = P0 + np.outer(t, d)
        return points

    target = horiz / drop

    def ratio(theta):
        denom = 1.0 - np.cos(theta)
        return (theta - np.sin(theta)) / denom

    def f(theta):
        return ratio(theta) - target

    # Brachistochrone branch uses theta in (0, 2*pi)
    lo = 1e-6
    hi = 2 * np.pi - 1e-6

    flo = f(lo)
    fhi = f(hi)

    if flo * fhi > 0:
        raise ValueError(
            f"Could not bracket a solution. target={target:.6f}, "
            f"f(lo)={flo:.6f}, f(hi)={fhi:.6f}"
        )

    for _ in range(max_iter):
        mid = 0.5 * (lo + hi)
        fmid = f(mid)

        if abs(fmid) < tol:
            theta_end = mid
            break

        if flo * fmid <= 0:
            hi = mid
            fhi = fmid
        else:
            lo = mid
            flo = fmid
    else:
        theta_end = 0.5 * (lo + hi)

    a = drop / (1.0 - np.cos(theta_end))

    theta = np.linspace(0.0, theta_end, num_points)
    x_local = a * (theta - np.sin(theta))
    y_local = a * (1.0 - np.cos(theta))

    points = P0 + np.outer(x_local, e_hat) + np.outer(y_local, g_hat)
    return points

def plot_vector(v):
    plt.quiver(v[0], v[1], np.sin(v[2]), np.cos(v[2]), color='green')

def main():
    # Set start and stop vectors
    # (pos_x, pos_y, heading_angle) 
    # Going up (+y) -> 0 degrees
    # Going right (+x) -> 90 degrees
    start = (0.0, 0.0, 0.0)
    goal = (-3.0, 2.0, -np.pi / 2)

    # Plotting Space
    plt.figure(figsize=(6, 6))
    plt.axis('equal')
    plt.xlim(-4, 1)
    plt.ylim(-1, 4)

    # Plot start and goal on grid
    plot_vector(start)
    plot_vector(goal)

    # Curves
    plt.title("Bezier Control Dist") 
    #pts = generate_hermite_curve(start, goal, num_points=15, scale=1.0)
    pts = generate_bezier_curve(start, goal, num_points=15, control_dist=1.25)
    plt.plot(pts[:,0], pts[:,1], 'b.', label = "bezier 1.25")

    pts = generate_bezier_curve(start, goal, num_points=15, control_dist=1.5)
    plt.plot(pts[:,0], pts[:,1], 'r.', label = "bezier 1.5")

    #pts = generate_brachistochrone(start[:2], goal[:2], g_vec=(0, 1))
    pts = generate_bezier_curve(start, goal, num_points=15, control_dist=1.75)
    plt.plot(pts[:,0], pts[:,1], 'g.', label = "bezier 1.75")

    plt.legend()
    plt.savefig("comp_bezier_ctrl_dist_2")


if __name__ == "__main__":
    main()