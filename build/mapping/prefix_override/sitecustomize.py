import sys
<<<<<<< HEAD
if sys.prefix == '/usr':
=======
if sys.prefix == '/home/group3-robot/ros2_venv':
>>>>>>> path-planner
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/group3-robot/robp_group3/install/mapping'
