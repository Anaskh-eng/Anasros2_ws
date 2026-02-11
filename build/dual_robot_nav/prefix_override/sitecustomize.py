import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/anaskh007/Anasros2_ws/install/dual_robot_nav'
