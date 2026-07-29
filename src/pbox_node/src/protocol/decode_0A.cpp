// #include "AG_Pbox/protocol_asensing.h"
// #include "AG_Pbox/TinyLog.h"
// #include "rclcpp/rclcpp.hpp"

#include "AGPbox.hpp"
#ifdef ROS1_PLATFORM
/* Asensing msg */
#include "pbox_node/Imu0A.h"
#include "pbox_node/ImuInitial.h"
#endif
#ifdef ROS2_PLATFORM
#include "imu_msgs/msg/imu0_a.hpp"
#include "imu_msgs/msg/imu_initial.hpp"
// /* ros2 msg */
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#endif

#define PROTOCOL_BDDB0A

class Decode_0A final : public ProtocolAsensing
{

public:
#ifdef ROS2_PLATFORM
    Decode_0A(std::shared_ptr<rclcpp::Node> node) : m_node(node)
    {
    // m_node->get_parameter("MsgType" , m_msgSwitch);
    // m_node->get_parameter("ProtocolType" , m_protocolType);
#endif

#ifdef ROS1_PLATFORM
    Decode_0A(std::shared_ptr<ros::NodeHandle> node) : m_node(node)
    {
    // m_node->get_parameter("MsgType" , m_msgSwitch);
    // m_node->get_parameter("ProtocolType" , m_protocolType);
        m_msgSwitch = m_node->param<int>("MsgType",1);
        m_protocolType = m_node->param<int>("ProtocolType",0);

#endif
        registProtocol(m_type , m_length , this);
    }

    ~Decode_0A(){}

    void subData(const uint8_t* sub_address, int& index)
    {
        
        int sub_index = 3;
        uint8_t check_sum = 0;
        int dataLength = getLength(m_type);
        /* check xor */
        for (int i = 0; i < dataLength -1; ++i)
        {
            check_sum ^= sub_address[i];
        }

        if (check_sum == sub_address[dataLength -1])
        {
            int16_t middle;
            uint32_t tmpU32;
            sub_index = 3;
            // gyro
            imu0a_data.gyro_x = toValue<float>(sub_address, sub_index);
            imu0a_data.gyro_y = toValue<float>(sub_address, sub_index);
            imu0a_data.gyro_z = toValue<float>(sub_address, sub_index);
            // acc
            imu0a_data.acc_x = toValue<float>(sub_address, sub_index);
            imu0a_data.acc_y = toValue<float>(sub_address, sub_index);
            imu0a_data.acc_z = toValue<float>(sub_address, sub_index);

            // temp
            middle = toValue<int16_t>(sub_address, sub_index);
            imu0a_data.temperature = middle * 200.0 / 32768;

            tmpU32 = toValue<uint32_t>(sub_address, sub_index);
            imu0a_data.time_stamp = (float)tmpU32/4000;     //for 570D 1/1000; other 1/4000

            publish_ros_msg();

            index += dataLength;
        }
        else
        {
            index += 3;
        }
    }

    //BDDB0A protocol define
    typedef struct _asensing_imu0a
    {
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float acc_x;
        float acc_y;
        float acc_z;
        float temperature;
        float time_stamp;
        uint8_t status;
        uint16_t frame_count;
    }asensing_imu0a_t;

private:
    void publish_ros_msg(void)
    {
        if(m_msgSwitch == 1)
        {
            /* Asensing msg type */
            m_msg0A.gx = m_msg.gx = imu0a_data.gyro_x;
            m_msg0A.gy = m_msg.gy = imu0a_data.gyro_y;
            m_msg0A.gz = m_msg.gz = imu0a_data.gyro_z;
            m_msg0A.ax = m_msg.ax = imu0a_data.acc_x;
            m_msg0A.ay = m_msg.ay = imu0a_data.acc_y;
            m_msg0A.gz = m_msg.az = imu0a_data.acc_z;
            m_msg0A.temperature = m_msg.temperature = imu0a_data.temperature;
            m_msg.imu_time_stamp = imu0a_data.time_stamp;

            if(m_protocolType == 1)
            {
                m_msg0A.status = imu0a_data.status;
                m_msg0A.frame_count = imu0a_data.frame_count;
                /* msg header */
                #ifdef ROS2_PLATFORM
                m_msg0A.header.frame_id = m_frameId;
                m_msg0A.header.stamp = rclcpp::Clock().now();
                if(!isCreate)
                {
                    isCreate = true;
                    m_pub0A = m_node->create_publisher<imu_msgs::msg::Imu0A>(m_topicName.c_str(), 500);
                }
                m_pub0A->publish(std::move(m_msg0A));
                #endif
                #ifdef ROS1_PLATFORM
                m_msg0A.header.stamp = ros::Time::now();
                m_msg0A.header.frame_id = m_frameId;
                if(!isCreate)
                {
                    isCreate = true;
                    m_pub = std::make_shared<ros::Publisher>(m_node->advertise<pbox_node::Imu0A>(m_topicName.c_str(), 2));
                }
                m_pub->publish(m_msg0A);
                #endif
            }
            else if(m_protocolType == 0)
            {
                /* msg header */
                #ifdef ROS2_PLATFORM
                m_msg.header.frame_id = m_frameId;
                m_msg.header.stamp = rclcpp::Clock().now();
                if(!isCreate)
                {
                    isCreate = true;
                    m_pub = m_node->create_publisher<imu_msgs::msg::ImuInitial>(m_topicName.c_str(), 500);
                }
                m_pub->publish(std::move(m_msg));
                #endif
                #ifdef ROS1_PLATFORM
                m_msg.header.stamp = ros::Time::now();
                m_msg.header.frame_id = m_frameId;
                if(!isCreate)
                {
                    isCreate = true;
                    m_pub = std::make_shared<ros::Publisher>(m_node->advertise<pbox_node::ImuInitial>(m_topicName.c_str(), 2));
                }
                m_pub->publish(m_msg);
                #endif
                // m_msg.imu_time_stamp = (toValue<uint32_t>(sub_address , sub_index)) * 0.001;
                // /* msg header */
                // m_msg.header.frame_id = m_frameId;
                // m_msg.header.stamp = rclcpp::Clock().now();
                // if(!isCreate)
                // {
                //     isCreate = true;
                //     m_pub = m_node->create_publisher<imu_msgs::msg::ImuInitial>(m_topicName.c_str(), 500);
                // }
                // m_pub->publish(std::move(m_msg));
            }
            else
            {
                TinyLog::info("protocol type error:%d" , m_protocolType);
            }
        }
        else if(m_msgSwitch == 0)
        {
            /* ros2 msg type */
            // static bool isCreate = false;
            // static sensor_msgs::msg::Imu pubMsg;
            // static rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
            // if(!isCreate)
            // {
            //     isCreate = true;
            //     publisher = m_node->create_publisher<sensor_msgs::msg::Imu>(m_topicName.c_str(), 500);
            // }
            // /* frame BDDB0A not have roll pitch yaw */

            // pubMsg.angular_velocity.x = toValue<float>(sub_address, sub_index);
            // pubMsg.angular_velocity.y = toValue<float>(sub_address, sub_index);
            // pubMsg.angular_velocity.z = toValue<float>(sub_address, sub_index);

            // pubMsg.linear_acceleration.x = toValue<float>(sub_address, sub_index);
            // pubMsg.linear_acceleration.y = toValue<float>(sub_address, sub_index);
            // pubMsg.linear_acceleration.z = toValue<float>(sub_address, sub_index);

            // /* msg header */
            // pubMsg.header.frame_id = m_frameId;
            // pubMsg.header.stamp = rclcpp::Clock().now();
            // publisher->publish(std::move(pubMsg));
        }
        else
        {
            static bool isPrint = false;
            if(!isPrint)
            {
                TinyLog::error("unknown msg type , please check your launch file!");
                isPrint = true;
            }
        }
    }

public:
    asensing_imu0a_t imu0a_data;

private:
    /* protocol info */
    std::string m_type = "BDDB0A";
    int m_length = 34;

    /* topic info */
    std::string m_topicName = "Imu0A";
    std::string m_frameId = "Imu0A";

    /* node */
    #ifdef ROS2_PLATFORM
    std::shared_ptr<rclcpp::Node> m_node;
    imu_msgs::msg::ImuInitial m_msg;
    imu_msgs::msg::Imu0A m_msg0A;
    rclcpp::Publisher<imu_msgs::msg::ImuInitial>::SharedPtr m_pub;
    rclcpp::Publisher<imu_msgs::msg::Imu0A>::SharedPtr m_pub0A;
    #endif

    #ifdef ROS1_PLATFORM
    std::shared_ptr<ros::NodeHandle> m_node;
    pbox_node::ImuInitial m_msg;
    pbox_node::Imu0A m_msg0A;
    std::shared_ptr<ros::Publisher> m_pub;
    #endif
    /* use ros msg or asensing msg */
    int m_msgSwitch = 1;
    int m_protocolType;
    bool isCreate = false; 
};