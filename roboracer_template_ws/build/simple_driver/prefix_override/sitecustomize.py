import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/patchychan/driver_template/roboracer_template_ws/install/simple_driver'
