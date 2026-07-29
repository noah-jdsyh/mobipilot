// #include "AG_Pbox/protocol_asensing.h"
// #include "AG_Pbox/TinyLog.h"
// #include "rclcpp/rclcpp.hpp"
#include "AGPbox.hpp"
#ifdef ROS1_PLATFORM
/* Asensing msg */
#include "pbox_node/ASENSING.h"
#include "pbox_node/Imu.h"
#endif

#ifdef ROS2_PLATFORM
/* Asensing msg */
#include "imu_msgs/msg/imu.hpp"
/* ros2 msg */
#include "sensor_msgs/msg/imu.hpp"
#endif

#define PROTOCOL_BDDB0B

class Decode_0B final : public ProtocolAsensing
{

public:
#ifdef ROS2_PLATFORM
    Decode_0B(std::shared_ptr<rclcpp::Node> node) : m_node(node)
    {
        m_node->get_parameter("MsgType" , m_msgSwitch);
        m_node->get_parameter("Grange0B" , m_grange);
        m_node->get_parameter("Arange0B" , m_arange);
#endif     
#ifdef ROS1_PLATFORM
    Decode_0B(std::shared_ptr<ros::NodeHandle> node) : m_node(node)
    {
        m_msgSwitch = m_node->param<int>("MsgType",1);
        m_grange = m_node->param<float>("Grange0B",4.0);
        m_arange = m_node->param<float>("Arange0B",4.0);

#endif
        registProtocol(m_typeImu , m_lengthImu , this);
    }

    ~Decode_0B(){}

    void subData(const uint8_t* sub_address, int& index)
    {
        std::string type = "";
		char str[16] = {0};
		sprintf(str , "%02X%02X%02X" , sub_address[0] ,
									   sub_address[1] , 
									   sub_address[2]);
		type += str;
        if(type == m_typeImu)
        {
            parse0B(sub_address , index);
        }
        else
        {
            index += 3;
            TinyLog::error("protocol type error:%s!" , type.c_str());
        }
    }

    void parse0B(const uint8_t* data, int& pos)
    {
        int sub_index = 3;
        uint8_t check_sum = 0;
        int dataLength = getLength(m_typeImu);
        /* check xor */
        for (int i = 0; i < dataLength -1; ++i)
        {
            check_sum ^= data[i];
        }

        if(check_sum != data[dataLength - 1])
        {
            pos += 4;
        }
        else
        {
            int16_t temp[3];
            /* roll pitch yaw */
            m_imuMsg.imu_msg.roll = (toValue<int16_t>(data, sub_index)) * 360.0 / 32768;
            m_imuMsg.imu_msg.pitch = (toValue<int16_t>(data, sub_index)) * 360.0 / 32768;
            m_imuMsg.imu_msg.azimuth = (toValue<int16_t>(data, sub_index)) * 360.0 / 32768;

            /* gx gy gz , ax ay az */
            m_imuMsg.imu_msg.x_angular_velocity = (toValue<int16_t>(data, sub_index)) * 300.0 / 32768;
            m_imuMsg.imu_msg.y_angular_velocity = (toValue<int16_t>(data, sub_index)) * 300.0 / 32768;
            m_imuMsg.imu_msg.z_angular_velocity = (toValue<int16_t>(data, sub_index)) * 300.0 / 32768;
            m_imuMsg.imu_msg.x_acc = (toValue<int16_t>(data, sub_index)) * 12.0 / 32768;
            m_imuMsg.imu_msg.y_acc = (toValue<int16_t>(data, sub_index)) * 12.0 / 32768;
            m_imuMsg.imu_msg.z_acc = (toValue<int16_t>(data, sub_index)) * 12.0 / 32768;

            m_imuMsg.imu_msg.latitude = (toValue<int32_t>(data, sub_index)) * 0.0000001;
            m_imuMsg.imu_msg.longitude = (toValue<int32_t>(data, sub_index)) * 0.0000001;
            m_imuMsg.imu_msg.altitude = (toValue<int32_t>(data, sub_index)) * 0.001;
            m_imuMsg.imu_msg.north_velocity = (toValue<int16_t>(data, sub_index)) * 100.0 / 32768;
            m_imuMsg.imu_msg.east_velocity = (toValue<int16_t>(data, sub_index)) * 100.0 / 32768;
            m_imuMsg.imu_msg.ground_velocity = (toValue<int16_t>(data, sub_index)) * 100.0 / 32768;
            
            m_imuMsg.imu_msg.ins_status = data[sub_index ++];
    
            sub_index += 4;
            sub_index += 2;
            temp[0] = toValue<int16_t>(data, sub_index);
            temp[1] = toValue<int16_t>(data, sub_index);
            temp[2] = toValue<int16_t>(data, sub_index);
            m_ts = (toValue<uint32_t>(data, sub_index)) / 4000.0;
            switch (data[sub_index++])
            {
            case 0:
                m_imuMsg.imu_msg.latitude_std = exp(temp[0]/100);
                m_imuMsg.imu_msg.longitude_std = exp(temp[1]/100);
                m_imuMsg.imu_msg.altitude_std = exp(temp[2]/100);
                break;
            case 1:
                m_imuMsg.imu_msg.north_velocity_std = exp(temp[0]/100);
                m_imuMsg.imu_msg.east_velocity_std = exp(temp[1]/100);
                m_imuMsg.imu_msg.ground_velocity_std = exp(temp[2]/100);
                break;
            case 2:
                m_imuMsg.imu_msg.roll_std = exp(temp[0]/100);
                m_imuMsg.imu_msg.pitch_std = exp(temp[1]/100);
                m_imuMsg.imu_msg.azimuth_std = exp(temp[2]/100);
                break;
            case 22:
                m_imuMsg.imu_msg.temperature = temp[0] * 200.0 / 32768;
                break;
            case 32:
                m_imuMsg.imu_msg.position_type = temp[0];
                m_imuMsg.imu_msg.numsv = temp[1];
                m_imuMsg.imu_msg.heading_type = temp[2];
                break;
            case 33:
                m_imuMsg.imu_msg.wheel_speed_status = temp[1];
                break;
            default:
                break;
            }
            m_imuMsg.imu_msg.sec_of_week = m_ts;
            sub_index++;    // check byte 
            m_imuMsg.imu_msg.gps_week_number = toValue<uint32_t>(data, sub_index);

            pos += m_lengthImu;
            #ifdef ROS2_PLATFORM
            /* roll pitch yaw to quaternion */
            double toQua[3];
            toQua[0] = m_imuMsg.imu_msg.roll * 6.283185307179586232 / 360.0;
            toQua[1] = m_imuMsg.imu_msg.pitch * 6.283185307179586232 / 360.0;
            toQua[2] = m_imuMsg.imu_msg.azimuth * 6.283185307179586232 / 360.0;
            double quater[4] = {0};
            this->toQuaternion(toQua , quater);
            m_rosImuMsg.orientation.x = quater[0];
            m_rosImuMsg.orientation.y = quater[1];
            m_rosImuMsg.orientation.z = quater[2];
            m_rosImuMsg.orientation.w = quater[3];

            m_rosImuMsg.angular_velocity.x = m_imuMsg.imu_msg.x_angular_velocity;
            m_rosImuMsg.angular_velocity.y = m_imuMsg.imu_msg.x_angular_velocity;
            m_rosImuMsg.angular_velocity.z = m_imuMsg.imu_msg.x_angular_velocity;
            m_rosImuMsg.linear_acceleration.x = m_imuMsg.imu_msg.x_acc;
            m_rosImuMsg.linear_acceleration.y = m_imuMsg.imu_msg.y_acc;
            m_rosImuMsg.linear_acceleration.z = m_imuMsg.imu_msg.z_acc;
            if(m_msgSwitch == 1)
            {
                static bool isCreate = false;
                static rclcpp::Publisher<imu_msgs::msg::Imu>::SharedPtr publisher;
                if(!isCreate)
                {
                    isCreate = true;
                    publisher = m_node->create_publisher<imu_msgs::msg::Imu>(m_topicNameImu.c_str(), 500);
                }
                m_imuMsg.header.frame_id = m_frameIdImu;
                m_imuMsg.header.stamp = rclcpp::Clock().now();
                publisher->publish(std::move(m_imuMsg));
            }
            else if(m_msgSwitch == 0)
            {
                static bool isCreate = false;
                static rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
                if(!isCreate)
                {
                    isCreate = true;
                    publisher = m_node->create_publisher<sensor_msgs::msg::Imu>(m_topicNameImu.c_str(), 500);
                }
                m_rosImuMsg.header.frame_id = m_frameIdImu;
                m_rosImuMsg.header.stamp = rclcpp::Clock().now();
                publisher->publish(std::move(m_rosImuMsg));
            }
            #endif

            #ifdef ROS1_PLATFORM
            if(m_msgSwitch == 1)
            {
                static bool isCreate = false;
                if(!isCreate)
                {
                    isCreate = true;
                    m_insimu_pub = std::make_shared<ros::Publisher>(m_node->advertise<pbox_node::Imu>(m_topicNameImu.c_str(), 2));
                }
                m_imuMsg.header.frame_id = m_frameIdImu;
                m_imuMsg.header.stamp = ros::Time::now();
                m_insimu_pub->publish(std::move(m_imuMsg));
            }
            else if(m_msgSwitch == 0)
            {
                // static bool isCreate = false;
                // static rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
                // if(!isCreate)
                // {
                //     isCreate = true;
                //     publisher = m_node->create_publisher<sensor_msgs::msg::Imu>(m_topicNameImu.c_str(), 500);
                // }
                // m_rosImuMsg.header.frame_id = m_frameIdImu;
                // m_rosImuMsg.header.stamp = rclcpp::Clock().now();
                // publisher->publish(std::move(m_rosImuMsg));
            }
            #endif
        }
    }

private:
    /* protocol info */
    std::string m_typeImu = "BDDB0B";
    int m_lengthImu = 58;
   
    /* topic info */
    std::string m_topicNameImu = "Ins";
    std::string m_frameIdImu = "Ins";
    /* use ros msg or asensing msg */
    int m_msgSwitch = 0;
    #ifdef ROS2_PLATFORM
     /* node */
    std::shared_ptr<rclcpp::Node> m_node;
    /* msg */
    sensor_msgs::msg::Imu m_rosImuMsg;
    imu_msgs::msg::Imu m_imuMsg;
    #endif

    #ifdef ROS1_PLATFORM
    std::shared_ptr<ros::NodeHandle> m_node;
    pbox_node::Imu m_imuMsg;
    std::shared_ptr<ros::Publisher> m_insimu_pub;
    #endif

    /* range */
    float m_grange;
    float m_arange;
    float m_ts = 0.0;
};