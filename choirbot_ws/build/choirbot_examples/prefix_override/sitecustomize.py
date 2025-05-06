import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robot4/Documents/Nishant_github_repo/AdvancedMechatronics/choirbot_ws/install/choirbot_examples'
