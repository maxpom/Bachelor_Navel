import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/max/Bachelor_Navel/NavelTrajectory/webots_ros/install/my_package'
