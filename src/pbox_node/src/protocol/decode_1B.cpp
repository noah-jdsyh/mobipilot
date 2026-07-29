// #include "AG_Pbox/protocol_asensing.h"
// #include "AG_Pbox/TinyLog.h"
// #include "rclcpp/rclcpp.hpp"
#include "AGPbox.hpp"
#ifdef ROS1_PLATFORM
/* Asensing msg */
#include "pbox_node/Odom.h"
#endif

#ifdef ROS2_PLATFORM
/* Asensing msg */
#include "imu_msgs/msg/odom.hpp"
/* ros2 msg */
#include "nav_msgs/msg/odometry.hpp"
#endif

#define PROTOCOL_BDDB1B

class Decode_1B final : public ProtocolAsensing
{

public:
#ifdef ROS2_PLATFORM
    Decode_1B(std::shared_ptr<rclcpp::Node> node) : m_node(node)
    {
        m_node->get_parameter("MsgType" , m_msgSwitch);
#endif     
#ifdef ROS1_PLATFORM
    Decode_1B(std::shared_ptr<ros::NodeHandle> node) : m_node(node)
    {
        m_msgSwitch = m_node->param<int>("MsgType",1);

#endif
        registProtocol(m_typeOdom , m_lengthOdom , this);
    }

    ~Decode_1B(){}

    void subData(const uint8_t* sub_address, int& index)
    {
        std::string type = "";
		char str[16] = {0};
		sprintf(str , "%02X%02X%02X" , sub_address[0] ,
									   sub_address[1] , 
									   sub_address[2]);
		type += str;
        if(type == m_typeOdom)
        {
            parse1B(sub_address , index);
        }
        else
        {
            index += 3;
            TinyLog::error("protocol type error:%s!" , type.c_str());
        }
    }

    void parse1B(const uint8_t* data, int& pos)
    {
        int sub_index = 3;
        uint8_t check_sum = 0;
        int dataLength = getLength(m_typeOdom);
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
            int16_t temp[3];
            uint8_t poll_type ; 
            uint8_t routing_cnt = 0;
            uint32_t ts_ms = 0;
            m_OdomMsg.q0_w = (toValue<int32_t>(data,sub_index)) * (1.0e-9);
            m_OdomMsg.q1_x = (toValue<int32_t>(data,sub_index)) * (1.0e-9);
            m_OdomMsg.q2_y = (toValue<int32_t>(data,sub_index)) * (1.0e-9);
            m_OdomMsg.q3_z = (toValue<int32_t>(data,sub_index)) * (1.0e-9);
            m_OdomMsg.pos_x = (toValue<int32_t>(data,sub_index)) * (1.0e-3);
            m_OdomMsg.pos_y = (toValue<int32_t>(data,sub_index)) * (1.0e-3);
            m_OdomMsg.pos_z = (toValue<int32_t>(data,sub_index)) * (1.0e-3);
            m_OdomMsg.vel_x = (toValue<int16_t>(data,sub_index)) * 100 / 32768.0;
            m_OdomMsg.vel_y = (toValue<int16_t>(data,sub_index)) * 100 / 32768.0;
            m_OdomMsg.vel_z = (toValue<int16_t>(data,sub_index)) * 100 / 32768.0;
            m_OdomMsg.vel = (toValue<int16_t>(data,sub_index)) * 100 / 32768.0;
            m_OdomMsg.ang_vel_x = (toValue<int16_t>(data,sub_index)) * 300 / 32768.0;
            m_OdomMsg.ang_vel_y = (toValue<int16_t>(data,sub_index)) * 300 / 32768.0;
            m_OdomMsg.ang_vel_z = (toValue<int16_t>(data,sub_index)) * 300 / 32768.0;
            m_OdomMsg.acc_x = (toValue<int16_t>(data,sub_index)) * 12.0 / 32768.0;
            m_OdomMsg.acc_y = (toValue<int16_t>(data,sub_index)) * 12.0 / 32768.0;
            m_OdomMsg.acc_z = (toValue<int16_t>(data,sub_index)) * 12.0 / 32768.0;
            m_OdomMsg.status = data[sub_index++];
            m_OdomMsg.sensor_status = (toValue<uint32_t>(data,sub_index));
            temp[0] = (toValue<int16_t>(data,sub_index));
            temp[1] = (toValue<int16_t>(data,sub_index));
            temp[2] = (toValue<int16_t>(data,sub_index));
            poll_type = data[sub_index++];
            routing_cnt = data[sub_index++];
            ts_ms = (toValue<int32_t>(data,sub_index))*0.25;
            m_OdomMsg.tow_ms = ts_ms;
            switch(poll_type)
            {
                //pos std
                case 0: 
                    m_OdomMsg.pos_x_std = (double)exp((double)temp[0]/100);
                    m_OdomMsg.pos_y_std = (double)exp((double)temp[1]/100);
                    m_OdomMsg.pos_z_std = (double)exp((double)temp[2]/100);
                break;
                //vel std
                case 1:
                    m_OdomMsg.vel_x_std = (double)exp((double)temp[0]/100);
                    m_OdomMsg.vel_y_std = (double)exp((double)temp[1]/100);
                    m_OdomMsg.vel_z_std = (double)exp((double)temp[2]/100);
                break;
                //att std
                case 2:
                    m_OdomMsg.roll_std = (double)exp((double)temp[0]/100);
                    m_OdomMsg.pitch_std = (double)exp((double)temp[1]/100);
                    m_OdomMsg.yaw_std = (double)exp((double)temp[2]/100);
                break;
                default:
                break;
            }
            pos += m_lengthOdom;
            publishOdomMsg();
        }
    }
    void publishOdomMsg(void)
    {
        #ifdef ROS1_PLATFORM
        if(m_msgSwitch == 1)
        {
            static bool isCreate = false;
            if(!isCreate)
            {
                isCreate = true;
                m_Odom_pub = std::make_shared<ros::Publisher>(m_node->advertise<pbox_node::Odom>(m_topicNameOdom.c_str(), 2));
            }
            m_OdomMsg.header.frame_id = m_frameIdOdom;
            m_OdomMsg.header.stamp = ros::Time::now();
            m_Odom_pub->publish(std::move(m_OdomMsg));
        }
        else if(m_msgSwitch == 0)
        {
        }
        #endif
        #ifdef ROS2_PLATFORM
        if(m_msgSwitch == 1)
        {
            static bool isCreate = false;
            static rclcpp::Publisher<imu_msgs::msg::Odom>::SharedPtr publisher;
            if(!isCreate)
            {
                isCreate = true;
                publisher = m_node->create_publisher<imu_msgs::msg::Odom>(m_topicNameOdom.c_str(), 500);
            }
            m_OdomMsg.header.frame_id = m_frameIdOdom;
            m_OdomMsg.header.stamp = rclcpp::Clock().now();
            publisher->publish(std::move(m_OdomMsg));
        }
        else if(m_msgSwitch == 0)
        {
            m_rosPboxOdom.pose.pose.orientation.x = m_OdomMsg.q1_x;
            m_rosPboxOdom.pose.pose.orientation.y = m_OdomMsg.q2_y;
            m_rosPboxOdom.pose.pose.orientation.z = m_OdomMsg.q3_z;
            m_rosPboxOdom.pose.pose.orientation.w = m_OdomMsg.q0_w;
            m_rosPboxOdom.pose.pose.position.x = m_OdomMsg.pos_x;
            m_rosPboxOdom.pose.pose.position.y = m_OdomMsg.pos_y;
            m_rosPboxOdom.pose.pose.position.z = m_OdomMsg.pos_z;
            m_rosPboxOdom.pose.covariance[0] = m_OdomMsg.pos_x_std;
            m_rosPboxOdom.pose.covariance[7] = m_OdomMsg.pos_y_std;
            m_rosPboxOdom.pose.covariance[14] = m_OdomMsg.pos_z_std;
            m_rosPboxOdom.pose.covariance[21] = m_OdomMsg.roll_std;
            m_rosPboxOdom.pose.covariance[28] = m_OdomMsg.pitch_std;
            m_rosPboxOdom.pose.covariance[35] = m_OdomMsg.yaw_std;

            m_rosPboxOdom.twist.twist.linear.x = m_OdomMsg.vel_x;
            m_rosPboxOdom.twist.twist.linear.y = m_OdomMsg.vel_y;
            m_rosPboxOdom.twist.twist.linear.z = m_OdomMsg.vel_z;

            m_rosPboxOdom.twist.twist.angular.x = m_OdomMsg.ang_vel_x;
            m_rosPboxOdom.twist.twist.angular.y = m_OdomMsg.ang_vel_y;
            m_rosPboxOdom.twist.twist.angular.z = m_OdomMsg.ang_vel_z;

            m_rosPboxOdom.twist.covariance[0] = m_OdomMsg.vel_x_std;
            m_rosPboxOdom.twist.covariance[7] = m_OdomMsg.vel_y_std;
            m_rosPboxOdom.twist.covariance[14] = m_OdomMsg.vel_z_std;

            m_rosPboxOdom.header.frame_id = m_frameIdOdom;
            m_rosPboxOdom.header.stamp = rclcpp::Clock().now();
            {
                static bool isCreate = false;
                static rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
                if(!isCreate)
                {
                    isCreate = true;
                    publisher = m_node->create_publisher<nav_msgs::msg::Odometry>(m_topicNameOdom.c_str(), 500);
                }
                m_OdomMsg.header.frame_id = m_frameIdOdom;
                m_OdomMsg.header.stamp = rclcpp::Clock().now();
                publisher->publish(std::move(m_rosPboxOdom));
            }
        }
        #endif

    }
private:
    /* protocol info */
    std::string m_typeOdom = "BDDB1B";
    int m_lengthOdom = 69;
   
    /* topic info */
    std::string m_topicNameOdom = "Odom";
    std::string m_frameIdOdom = "Odom";
    
    /* use ros msg or asensing msg */
    int m_msgSwitch = 0;


    #ifdef ROS2_PLATFORM
     /* node */
    std::shared_ptr<rclcpp::Node> m_node;
    /* msg */
    nav_msgs::msg::Odometry m_rosPboxOdom;
    imu_msgs::msg::Odom m_OdomMsg;
    #endif

    #ifdef ROS1_PLATFORM
    std::shared_ptr<ros::NodeHandle> m_node;
    pbox_node::Odom m_OdomMsg;
    std::shared_ptr<ros::Publisher> m_Odom_pub;
    #endif

};