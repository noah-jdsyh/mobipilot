// #include "AG_Pbox/protocol_asensing.h"
// #include "AG_Pbox/TinyLog.h"
// #include "rclcpp/rclcpp.hpp"
#include "AGPbox.hpp"
#ifdef ROS1_PLATFORM
/* Asensing msg */
#include "pbox_node/Gnss.h"
#endif

#ifdef ROS2_PLATFORM
/* Asensing msg */
#include "imu_msgs/msg/gnss.hpp"
/* ros2 msg */
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#endif

#define PROTOCOL_BDDB10

class Decode_10 final : public ProtocolAsensing
{

public:
#ifdef ROS2_PLATFORM
    Decode_10(std::shared_ptr<rclcpp::Node> node) : m_node(node)
    {
        m_node->get_parameter("MsgType" , m_msgSwitch);
#endif     
#ifdef ROS1_PLATFORM
    Decode_10(std::shared_ptr<ros::NodeHandle> node) : m_node(node)
    {
        m_msgSwitch = m_node->param<int>("MsgType",1);

#endif
        registProtocol(m_typeGnss , m_lengthGnss , this);
    }

    ~Decode_10(){}

    void subData(const uint8_t* sub_address, int& index)
    {
        std::string type = "";
		char str[16] = {0};
		sprintf(str , "%02X%02X%02X" , sub_address[0] ,
									   sub_address[1] , 
									   sub_address[2]);
		type += str;
        if(type == m_typeGnss)
        {
            parseGnss(sub_address , index);
        }
        else
        {
            index += 3;
            TinyLog::error("protocol type error:%s!" , type.c_str());
        }
    }
    void parseGnss(const uint8_t* data, int& pos)
    {
        int sub_index = 3;
        uint8_t check_sum = 0;
        int dataLength = getLength(m_typeGnss);
        /* check xor */
        for (int i = 0; i < dataLength -1; ++i)
        {
            check_sum ^= data[i];
        }

        if(check_sum != data[dataLength - 1])
        {
            pos += 3;
        }
        else
        {
            m_gnssMsg.longitude         = (toValue<int32_t>(data, sub_index)) * 0.0000001;
            m_gnssMsg.lon_sigma         = (toValue<int16_t>(data, sub_index)) * 0.001;
            m_gnssMsg.latitude          = (toValue<int32_t>(data, sub_index)) * 0.0000001;
            m_gnssMsg.lat_sigma         = (toValue<int16_t>(data, sub_index)) * 0.001;
            m_gnssMsg.altitude          = (toValue<int32_t>(data, sub_index)) * 0.001;
            m_gnssMsg.alt_sigma         = (toValue<int16_t>(data, sub_index)) * 0.001;

            // m_gnssMsg.gps_fix           = toValue<uint16_t>(data , sub_index);  // sAcc
            sub_index += 2;

            // m_gnssMsg.rtk_age           = toValue<uint16_t>(data , sub_index);  //age & type
            m_gnssMsg.rtk_age           = data[sub_index++];  //age 
            sub_index ++;               //msg type

            m_gnssMsg.flags_pos         = data[sub_index++];
            m_gnssMsg.gps_fix           = m_gnssMsg.flags_pos;

            m_gnssMsg.flags_vel         = data[sub_index++];
            m_gnssMsg.flags_attitude    = data[sub_index++];
            m_gnssMsg.flags_time        = data[sub_index++];

            m_gnssMsg.hor_vel           = (toValue<int16_t>(data, sub_index))  * 0.01;
            m_gnssMsg.track_angle       = (toValue<int16_t>(data, sub_index))  * 0.01;
            m_gnssMsg.ver_vel           = (toValue<int16_t>(data, sub_index))  * 0.01;
            m_gnssMsg.latency_vel       = (toValue<int16_t>(data, sub_index))  * 0.001;
            m_gnssMsg.base_length       = (toValue<int16_t>(data, sub_index))  * 0.001;

            m_gnssMsg.yaw               = (toValue<int16_t>(data, sub_index))  * 0.01;
            m_gnssMsg.yaw_sigma         = (toValue<int16_t>(data, sub_index))  * 0.001;
            m_gnssMsg.pitch             = (toValue<int16_t>(data, sub_index))  * 0.001;
            m_gnssMsg.pitch_sigma       = (toValue<int16_t>(data, sub_index))  * 0.001;

            std::string timeStr;
            timeStr += std::to_string(((toValue<uint16_t>(data, sub_index) & 0x3F) + 2000));
            timeStr += "-";
            timeStr += std::to_string(data[sub_index++]);
            timeStr += "-";
            timeStr += std::to_string(data[sub_index++]);
            timeStr += " ";
            timeStr += std::to_string(data[sub_index++]);
            timeStr += ":";
            timeStr += std::to_string(data[sub_index++]);
            timeStr += ":";
            timeStr += std::to_string(toValue<uint16_t>(data, sub_index)*0.001);
            m_gnssMsg.utc_time = timeStr;

            m_gnssMsg.ts_pos            = toValue<uint32_t>(data, sub_index);
            m_gnssMsg.ts_vel            = toValue<uint32_t>(data, sub_index);
            m_gnssMsg.ts_heading        = toValue<uint32_t>(data, sub_index);
            m_gnssMsg.state             = data[sub_index++];
            m_gnssMsg.num_master        = data[sub_index++];
            sub_index++;
            sub_index++;
            m_gnssMsg.gdop              = (toValue<int16_t>(data, sub_index))  * 0.01;
            m_gnssMsg.pdop              = (toValue<int16_t>(data, sub_index))  * 0.01;
            m_gnssMsg.hdop              = (toValue<int16_t>(data, sub_index))  * 0.01;
            m_gnssMsg.htdop             = (toValue<int16_t>(data, sub_index))  * 0.01;
            m_gnssMsg.tdop              = (toValue<int16_t>(data, sub_index))  * 0.01;
            m_gnssMsg.num_reserve       = data[sub_index++];
            
            pos += m_lengthGnss;

            #ifdef ROS2_PLATFORM
            m_rosGnssMsg.latitude       = m_gnssMsg.latitude ;
            m_rosGnssMsg.longitude      = m_gnssMsg.longitude;
            m_rosGnssMsg.altitude       = m_gnssMsg.altitude;
            
            if(m_msgSwitch == 1)
            {
                static bool isCreate = false;
                static rclcpp::Publisher<imu_msgs::msg::Gnss>::SharedPtr publisher;
                if(!isCreate)
                {
                    isCreate = true;
                    publisher = m_node->create_publisher<imu_msgs::msg::Gnss>(m_topicNameGnss.c_str(), 500);
                }
                m_gnssMsg.header.frame_id = m_frameIdGnss;
                m_gnssMsg.header.stamp = rclcpp::Clock().now();
                publisher->publish(std::move(m_gnssMsg));
            }
            else if(m_msgSwitch == 0)
            {
                static bool isCreate = false;
                static rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher;
                if(!isCreate)
                {
                    isCreate = true;
                    publisher = m_node->create_publisher<sensor_msgs::msg::NavSatFix>(m_topicNameGnss.c_str(), 500);
                }
                m_rosGnssMsg.header.frame_id = m_frameIdGnss;
                m_rosGnssMsg.header.stamp = rclcpp::Clock().now();
                publisher->publish(std::move(m_rosGnssMsg));
            }
            #endif
            #ifdef ROS1_PLATFORM
            if(m_msgSwitch == 1)
            {
                static bool isCreate = false;
                if(!isCreate)
                {
                    isCreate = true;
                    m_gnss_pub = std::make_shared<ros::Publisher>(m_node->advertise<pbox_node::Gnss>(m_topicNameGnss.c_str(), 2));
                }
                m_gnssMsg.header.frame_id = m_frameIdGnss;
                m_gnssMsg.header.stamp = ros::Time::now();
                m_gnss_pub->publish(std::move(m_gnssMsg));
            }
            else if(m_msgSwitch == 0)
            {
                static bool isCreate = false;
                // if(!isCreate)
                // {
                //     isCreate = true;
                //     publisher = m_node->create_publisher<sensor_msgs::msg::NavSatFix>(m_topicNameGnss.c_str(), 500);
                // }
                // m_rosGnssMsg.header.frame_id = m_frameIdGnss;
                // m_rosGnssMsg.header.stamp = rclcpp::Clock().now();
                // publisher->publish(std::move(m_rosGnssMsg));
            }
            #endif
        }

    }

private:
    /* protocol info */
    std::string m_typeGnss = "BDDB10";
    int m_lengthGnss = 70;
   
    /* topic info */
    std::string m_topicNameGnss = "Gnss";
    std::string m_frameIdGnss = "Gnss";
    /* use ros msg or asensing msg */
    int m_msgSwitch = 0;
    #ifdef ROS2_PLATFORM
     /* node */
    std::shared_ptr<rclcpp::Node> m_node;
    /* msg */
    sensor_msgs::msg::NavSatFix m_rosGnssMsg;
    imu_msgs::msg::Gnss m_gnssMsg;
    #endif

    #ifdef ROS1_PLATFORM
    std::shared_ptr<ros::NodeHandle> m_node;
    pbox_node::Gnss m_gnssMsg;
    std::shared_ptr<ros::Publisher> m_gnss_pub;
    #endif

    /* range */
    float m_ts = 0.0;
};