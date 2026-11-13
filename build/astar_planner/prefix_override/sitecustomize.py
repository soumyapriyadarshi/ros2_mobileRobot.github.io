import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sipun/ros2_mobileRobot.github.io/install/astar_planner'
