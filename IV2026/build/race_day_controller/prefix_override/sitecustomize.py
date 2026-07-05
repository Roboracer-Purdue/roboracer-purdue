import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/patchy/arc_ws/ARC-PI-ROSCODE/IV2026/install/race_day_controller'
