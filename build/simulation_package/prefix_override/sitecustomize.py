import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/marcel/studies/2_sem_mgr/RPE_lab/EX6/ros2_ws/install/simulation_package'
