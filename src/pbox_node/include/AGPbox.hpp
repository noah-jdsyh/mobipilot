#ifndef _AG_Pbox_HPP_
#define _AG_Pbox_HPP_

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <memory>
#include <cstring>
#include <cmath>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <serial/serial.h>

#ifdef ROS2_PLATFORM
#include "rclcpp/rclcpp.hpp"
#endif

#ifdef ROS1_PLATFORM
#include <ros/ros.h>
#include <ros/package.h>
#endif

#include "protocol_asensing.h"
#include "TinyLog.h"
using namespace std;
using namespace std::chrono_literals;
using namespace serial;


#define UART_PORT       "/dev/ttyUSB0"
#define UART_BAUDRATE   (115200)
#ifdef ROS1_PLATFORM
class AGPbox : public ros::NodeHandle
#endif
#ifdef  ROS2_PLATFORM
class AGPbox : public rclcpp::Node
#endif
{
    public:
        AGPbox();
        void init_pbox_node();
    private:
        void timer_callback();
        void openSerialPort();
        void readSerialPort();

        void bindUdp();
        void readUdp();
    private:
        /* serial port info */
        serial::Serial imu_ser;
        std::string m_port;
        int m_baud;
        int m_UsbLatencyTime;

        /* UDP info */
        int m_serverSocket;
        std::string m_udpAddr;
        int m_udpPort;
        struct sockaddr_in m_server;
        socklen_t m_sockaddrLen;
        bool m_isBind;
        char readBuffer[1024];

        /* read cache */
        std::string m_input;
        std::string m_read;

        int m_connectionType;
        #ifdef  ROS1_PLATFORM
        std::shared_ptr<ros::Timer> timer_;
        #endif

        #ifdef  ROS2_PLATFORM
        rclcpp::TimerBase::SharedPtr timer_;
        #endif
        ProtocolAsensing connect_manager;
        /* imu log */
        bool m_isPrintLog = false;
        FILE* m_logFd = nullptr;
};
#endif