import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/st/py_ywfbot_ws_v1.1/install/ui_pack'
