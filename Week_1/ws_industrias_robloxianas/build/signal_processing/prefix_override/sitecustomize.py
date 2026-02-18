import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/reiv/workspace/MR_week1/Challenge_1/install/signal_processing'
