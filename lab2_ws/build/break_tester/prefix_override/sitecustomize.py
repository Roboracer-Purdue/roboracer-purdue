import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/patchychan/arc_ws/ARC-PI-ROSCODE/lab2_ws/install/break_tester'
