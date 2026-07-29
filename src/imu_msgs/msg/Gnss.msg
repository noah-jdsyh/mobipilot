std_msgs/Header header
float64 longitude
float32 lon_sigma
float64 latitude
float32 lat_sigma
float64 altitude
float32 alt_sigma

uint16 gps_fix
uint16 rtk_age
uint8 flags_pos
uint8 flags_vel
uint8 flags_attitude
uint8 flags_time

float32 hor_vel
float32 track_angle
float32 ver_vel
float32 latency_vel
float32 base_length

float32 yaw
float32 yaw_sigma
float32 pitch
float32 pitch_sigma

string utc_time
uint32 ts_pos
uint32 ts_vel
uint32 ts_heading
uint8 state
uint8 num_master

float32 gdop
float32 pdop
float32 hdop
float32 htdop
float32 tdop
uint8 num_reserve