import sys
if sys.prefix == '/home/user/.espressif/python_env/idf5.4_py3.10_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/ros2_ws/src/install/map_location_switch'
