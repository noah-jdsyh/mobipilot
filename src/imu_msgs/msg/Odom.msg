std_msgs/Header header
#四元数q0~q3
float32 q0_w            
float32 q1_x
float32 q2_y
float32 q3_z
#位置坐标
float32 pos_x
float32 pos_y
float32 pos_z
#速度
float32 vel_x
float32 vel_y
float32 vel_z
#XYZ3个方向合速度
float32 vel
#角速度
float32 ang_vel_x
float32 ang_vel_y
float32 ang_vel_z
#加速度
float32 acc_x
float32 acc_y
float32 acc_z
#odom状态
uint8 status
#传感器状态
uint32 sensor_status
#位置信息精度
float32 pos_x_std
float32 pos_y_std
float32 pos_z_std
#速度信息精度
float32 vel_x_std
float32 vel_y_std
float32 vel_z_std
#姿态信息精度
float32 roll_std
float32 pitch_std
float32 yaw_std
#周内秒毫秒
uint32 tow_ms



