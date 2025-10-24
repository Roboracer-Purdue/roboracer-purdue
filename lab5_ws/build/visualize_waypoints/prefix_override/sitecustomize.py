import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/patchy/arc_ws/ARC-PI-ROSCODE/lab5_ws/install/visualize_waypoints'
