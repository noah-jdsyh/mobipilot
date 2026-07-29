# 底盘控制指令  vb300_interfaces/ChassisCmd

float32 steer_angle      # 前轮转角 [rad], 左正右负
float32 throttle_pct     # 油门深度 [0-100] %
float32 brake_pct        # 制动液压力 [0-100]
uint8   gear             # 0=空挡  1=D挡前进  2=R挡倒车
bool    auto_enable      # 自动驾驶使能
bool    epb_release      # true=释放驻车  false=拉紧驻车
bool    left_signal      # 左转向灯
bool    right_signal     # 右转向灯
bool    horn             # 喇叭
bool    hazard_warning   # 危险警告灯
bool    reversing_light  # 倒车灯
bool    braking_light    # 制动灯
