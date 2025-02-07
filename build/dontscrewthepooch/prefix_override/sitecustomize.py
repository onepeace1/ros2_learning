import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/acai0804/ys25_ws/install/dontscrewthepooch'
