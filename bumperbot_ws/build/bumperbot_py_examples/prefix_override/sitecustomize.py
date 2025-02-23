import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/raz/projects/self_driving_OC_project/bumperbot_ws/install/bumperbot_py_examples'
