std_msgs/Header header

uint8 level           # 1/2/3 告警档；0=退出(恢复安全区)
string joint_name
float64 pos_act       # 当前实际位置(执行器空间, rad)
float64 threshold     # 触发的那档阈值(rad)
uint8 direction       # 0=下侧(min), 1=上侧(max), 2=none(退出)
