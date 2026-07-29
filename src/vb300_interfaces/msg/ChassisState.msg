# 底盘全部反馈  vb300_interfaces/ChassisState

float32 speed            # 实际车速 [m/s]
float32 steer_angle      # 实际前轮转角 [rad]
float32 brake_pressure   # 实际制动液压力 [0-100]
uint8   gear              # 实际档位  0=空 1=D 2=R
bool    auto_active       # 自动驾驶是否已激活
bool    motor_ready       # 电机控制器就绪
bool    epb_applied       # 驻车是否拉紧
bool    steer_takenover   # 方向盘被人工接管(应立即退出自动驾驶)
bool    eps_fault         # EPS 转向故障
uint8   ehb_fault_level   # EHB 制动故障等级 0~4, 0=正常
float32 battery_soc       # 动力电池电量 [0-100] %
float32 battery_voltage   # 动力电池电压 [V]
float32 battery_current   # 动力电池电流 [A]  正=放电 负=充电
float32 battery_temp      # 动力电池温度 [°C]
bool    high_voltage_on   # 高压是否接通
bool    is_charging       # 是否充电中
bool    dcdc_active       # DCDC 转换器工作状态
uint8   bms_error         # BMS 故障码
