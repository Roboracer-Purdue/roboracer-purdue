import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspaces/roboracer-purdue/roboracer_ws/install/f1tenth_gym_ros'
