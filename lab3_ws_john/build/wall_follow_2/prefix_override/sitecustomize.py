import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/patchy/arc_ws/ARC-PI-ROSCODE/lab3_ws_john/install/wall_follow_2'
