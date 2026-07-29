"""Launch a talker and a listener in a component container."""
 
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
 
 
def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            package="pbox_node",
            executable="pbox_pub",
            namespace='',
            name="pbox_pub",
            output="screen",
  
            #AG_Pbox 节点运行配置参数
            parameters=[
                #输出msg类型：ros2标准msg：0 , Asensing msg：1 
                #default: 1 Asensing msg 
                {"MsgType": 1},

                #连接类型：serial port：0 , UDP：1
                #default: 0 serial port
                {"ConnectionType": 1},
                
				#串口设备串  defaule: /dev/ttyUSB0
                {"UART_Port": "/dev/ttyUSB0"},
				#串口波特率  default: 115200
                {"UART_Baudrate": 230400},
                #latency_timer :1 ~ 16, default:16 
                {"USB_LatencyTime": 16},

                #UDP addr  default 192.168.225.2
                {"UDP_Addr": "192.168.225.2"},
                #UDP port  default 12300
                {"UDP_Port": 12300},

                #IMU BDDB0A协议类型 0：V1.0  1：V2.0----072
                {"ProtocolType":0},
				#5503协议：陀螺量程
                {"Grange04": 250.0},
				#AG041协议：加表量程
                {"Arange04": 4.0},
                
				#570D协议：陀螺量程
				{"Grange0B": 250.0},
				#570D协议：加表量程
				{"Arange0B": 4.0},

                #设置日志文件路径和文件名
                #default: close
                {"LogInfo":"./debug.log"},
                #设置日志打印等级：DEBUG:0 (save imu rawdata),INFO:1,WARNING:2,ERROR:3,FATAL:4
                #default:INFO
                {"LogLevel":1}
            ],
    )
    return launch.LaunchDescription([container])