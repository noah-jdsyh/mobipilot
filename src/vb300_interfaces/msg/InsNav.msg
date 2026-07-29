# InsNav.msg — 循迹用组合导航定位消息
# 由 ins_nav_node 订阅官方驱动的 imu_msgs/Imu (/Ins) 转换发布
# 坐标约定: 局部 ENU (x东 y北 z高), 原点为首帧有效定位 (或 origin_lat/lon 参数)
# 角度约定: yaw 为 ENU 标准 (东起逆时针), 已由协议"正北顺时针"换算

std_msgs/Header header          # frame_id='map'

# ---- 原始 WGS84 ----
float64 latitude                # deg
float64 longitude               # deg
float64 altitude                # m

# ---- 局部 ENU 平面坐标 (UTM 投影后减原点) ----
float64 x                       # m 东
float64 y                       # m 北
float64 z                       # m 高

# ---- 姿态 (rad) ----
float64 roll
float64 pitch
float64 yaw                     # ENU 东起逆时针

# ---- 速度 ----
float64 speed                   # 水平合速度 m/s
float64 vn                      # 北向速度 m/s
float64 ve                      # 东向速度 m/s
float64 vd                      # 地向速度 m/s
float64 yaw_rate                # 偏航角速度 rad/s

# ---- 状态 ----
uint8 status                    # 0未初始化 1对准中 2单点 3伪距差分 4RTK浮点 5RTK固定 6纯惯导 7故障
uint8 position_type             # 导远附录一原始解状态 (48/49/50=RTK固定)
uint8 numsv                     # 卫星数
bool aligned                    # 位置初对准完成 (false 时上述定位字段无效)
