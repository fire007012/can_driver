uint16 motor_id  # 电机 ID（总线节点号）
string name      # 关节/电机名称
uint8 mode       # 运行模式，见下方 MODE_* 常量
bool enabled     # 使能状态
bool fault       # 故障标志
int32 position   # 位置反馈（协议原始单位）
int16 velocity   # 速度反馈（协议原始单位）
int16 current    # 电流反馈（协议原始单位）

# Common mode constants so topics/services share the same enum semantics.
uint8 MODE_UNKNOWN=0
uint8 MODE_POSITION=1
uint8 MODE_VELOCITY=2
