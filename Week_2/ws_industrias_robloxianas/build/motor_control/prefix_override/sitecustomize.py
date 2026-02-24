import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/reiv/workspace/MR_week2/Challenge_2/install/motor_control'
