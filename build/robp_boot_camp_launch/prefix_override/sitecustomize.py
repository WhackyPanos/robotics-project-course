import sys
if sys.prefix == '/home/group3-robot/ros2_venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/group3-robot/robp_group3/install/robp_boot_camp_launch'
