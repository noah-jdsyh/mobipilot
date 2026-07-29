#include <chrono>
#include "AGPbox.hpp"
#include <utility>
#include "TinyLog.h"
#include "protocol_asensing.h"

const std::string copyright = \
"\n*******************************************************************************\n\
                    Copyright (C) Asensing (2024).                              \n\
All rights reserved. This software is Asensing property. Duplication or         \n\
disclosure without Asensing written authorization is prohibited.                \n\
********************************************************************************\n\
@Node Description   : ROS1 node dirver for Asensing Pbox                        \n\
@Author             : yfsw email:yfsw@asensing.com                              \n\
@Version            : V3.0.5                                                    \n\
@Release_Date       : 2024-04-12                                                \n\
********************************************************************************";

#ifdef ROS1_PLATFORM
AGPbox :: AGPbox() : NodeHandle("~")
#endif
#ifdef  ROS2_PLATFORM
AGPbox :: AGPbox() : Node("pbox_node")
#endif
{

}

void AGPbox::init_pbox_node()
{
    std::string logInfo = "";
    int32_t logLevel = 1;

    TinyLog::info("pbox_node");
    #ifdef ROS1_PLATFORM
    m_connectionType = this->param<int>("ConnectionType",1);
    m_port = this->param<std::string>("UART_Port",UART_PORT);
    m_baud = this->param<int>("UART_Baudrate",UART_BAUDRATE);
    m_udpAddr = this->param<std::string>("UDP_Addr","192.168.225.2");
    m_udpPort = this->param<int>("UDP_Port",12300);
    m_UsbLatencyTime = this->param<int>("USB_LatencyTime",16);

    logInfo = this->param<std::string>("LogInfo","./debug.log");
    logLevel = this->param<int>("LogLevel",0);
    timer_ = std::make_shared<ros::Timer>(this->createTimer(ros::Duration(0.001), std::bind(&AGPbox::timer_callback,this)));
    std::string path = ros::package::getPath("pbox_node") + "/ImuDebug.log";
    logInfo.erase(0, logInfo.find_first_not_of('.'));
    logInfo.erase(logInfo.find_last_not_of('.') + 1);
    logInfo = ros::package::getPath("pbox_node") + logInfo;

    #endif

    #ifdef  ROS2_PLATFORM
    /* all parameter declare and init here */
    this->declare_parameter<int>("MsgType", 1);
    this->declare_parameter<int>("ConnectionType", 0);
    /* serial port */
    this->declare_parameter<std::string>("UART_Port", UART_PORT);
    this->declare_parameter<int>("UART_Baudrate", UART_BAUDRATE);
    /* UDP */
    this->declare_parameter<std::string>("UDP_Addr", "192.168.119.133");
    this->declare_parameter<int>("UDP_Port", 12300);
    this->declare_parameter<int>("USB_LatencyTime" , 16);
    /* log */
    this->declare_parameter<std::string>("LogInfo", "");
    this->declare_parameter<int>("LogLevel", 1);
    /* range */
    this->declare_parameter<float>("Grange04", 250.0);
    this->declare_parameter<float>("Arange04", 4.0);
    this->declare_parameter<float>("Grange0B", 4.0);
    this->declare_parameter<float>("Arange0B", 4.0);

    /* init parameter */
    this->get_parameter("ConnectionType" , m_connectionType);
    this->get_parameter("UART_Port",m_port);
    this->get_parameter("UART_Baudrate",m_baud);
    this->get_parameter("UDP_Addr" , m_udpAddr);
    this->get_parameter("UDP_Port" , m_udpPort);
    this->get_parameter("USB_LatencyTime" , m_UsbLatencyTime);

    /* set log config */

    this->get_parameter("LogInfo",logInfo);
    this->get_parameter("LogLevel",logLevel);
    timer_ = this->create_wall_timer(1ms, std::bind(&AGPbox::timer_callback, this));
    std::string path = "./ImuDebug.log";
    #endif
    TinyLog::info("LogPathInfo: %s",logInfo.c_str());
    TinyLog::setStorageDir(logInfo.c_str());
    TinyLog::setStorageLevel(logLevel);
    TinyLog::info("%s",copyright.c_str());
    if(logLevel == 0)
    {
        m_isPrintLog = true;
        m_logFd = fopen(path.c_str(), "wb+");
        if (m_logFd == nullptr)
        {
            TinyLog::error("open log path fail,please check your launch file!");
        }
        TinyLog::info("open log path : %s ",path.c_str());
    }
    if(0 == m_connectionType)
    {
        openSerialPort();
    }
    else if(1 == m_connectionType)
    {
        bindUdp();
    }
}

void AGPbox::timer_callback()
{
    if(0 == m_connectionType)
    {
        readSerialPort();
    }
    else if(1 == m_connectionType)
    {
        readUdp();
    }
}
void AGPbox::openSerialPort()
{
    try
    {
        if((m_UsbLatencyTime < 16) && (m_UsbLatencyTime >= 1))
        {
            char command[128] = {0};
            sprintf(command , "echo %d > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer" , m_UsbLatencyTime);
            system("sudo chmod 777 /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");
            system((const char*)command);
            system("sync");
        }

        imu_ser.setPort(m_port);
        imu_ser.setBaudrate(m_baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(0);
        imu_ser.setTimeout(to);
        imu_ser.setFlowcontrol(serial::flowcontrol_t::flowcontrol_hardware);
        imu_ser.open();
    }
    catch (serial::IOException& e)
    {
        TinyLog::error("imu_serial opened failed! please check your connection!");
    }

    if (imu_ser.isOpen())
    {
        TinyLog::info("imu_serial opened successfully!");
    }
    else 
    {
        static bool isFirstOpen = true;
        if(isFirstOpen)
        {
            isFirstOpen = false;
            exit(0);
        }
    }
}
void AGPbox::readSerialPort()
{
    try
    {
        if (imu_ser.isOpen())
        {
            if(imu_ser.available())
            {
                m_read = imu_ser.read(imu_ser.available());
                m_input += m_read;
                connect_manager.addData(m_input);
                m_input.erase(0, m_input.size());
                if(m_isPrintLog)
                {
                    // for(int i = 0; i < m_read.size();i++)
                    {
                        // fprintf(m_logFd , "%02X" , m_read.c_str()[i]);
                        fwrite(m_read.c_str() , m_read.size() , 1u , m_logFd);
                    }
                }
            }
        }
        else
        {
            openSerialPort();
        }
    }
    catch (serial::IOException& e)
    {
        TinyLog::error("Error reading from the serial port:%s close port!" , imu_ser.getPort().c_str());
        imu_ser.close();
    }
}

void AGPbox::bindUdp()
{
    m_serverSocket = socket(AF_INET, SOCK_DGRAM, 0);
    (void)fcntl(m_serverSocket, F_SETFL, fcntl(m_serverSocket, F_GETFL, 0)|O_NONBLOCK);
    
    m_server.sin_family = AF_INET;
    m_server.sin_addr.s_addr = inet_addr(m_udpAddr.c_str());
    printf("%s : %d",m_udpAddr.c_str(),m_udpPort);
    m_server.sin_port = htons(m_udpPort);
    
    if(bind(m_serverSocket, (sockaddr *)&m_server, sizeof(sockaddr)) < 0)
    {
        (void)close(m_serverSocket);
        TinyLog::error("UDP bind fail!");
        m_isBind = false;
    }
    else
    {
        m_isBind = true;
        TinyLog::info("UDP bind success!");
    }
}
void AGPbox::readUdp()
{
    if(m_isBind)
    {
        m_sockaddrLen = sizeof(sockaddr);
        int readLen = 0;
        if(m_sockaddrLen)
        {
            do
            {
                memset(readBuffer , 0 , sizeof(readBuffer));
                readLen = recvfrom(m_serverSocket , readBuffer, 1024*sizeof(char), 0, (sockaddr *)&m_server, &m_sockaddrLen);
                
                if (readLen > 0)
                {
                    // printf("%d\n" , readLen);
                    connect_manager.addData(std::string((char*)readBuffer, readLen));
                    if(m_isPrintLog)
                    {
                        // for(int i = 0; i < readLen;i++)
                        {
                            // fprintf(m_logFd , "%02X" , readBuffer[i]);
                            fwrite(readBuffer , readLen , 1u , m_logFd);
                        }
                    }
                }
            }while (readLen > 0);
        }
    }
}
