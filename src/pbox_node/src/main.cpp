// #include "AG_Pbox.hpp"
#include "AGPbox.hpp"

#include "./protocol/decode_0A.cpp"
#include "./protocol/decode_0B.cpp"
#include "./protocol/decode_10.cpp"
#include "./protocol/decode_1B.cpp"

int main(int argc, char** argv)
{
    #ifdef ROS2_PLATFORM
    //初始化节点
    rclcpp::init(argc, argv);
    #endif


    #ifdef ROS1_PLATFORM
    ros::init(argc, argv, "pbox_pub");
    #endif

    auto node = std::make_shared<AGPbox>();
    node->init_pbox_node();
    /* create publisher */
    #ifdef PROTOCOL_BDDB0A
    auto publish0A = std::make_shared<Decode_0A>(node);
    #endif
    #ifdef PROTOCOL_BDDB04
    auto publish04 = std::make_shared<Decode_04>(node);
    #endif
    #ifdef PROTOCOL_BDDB8B
    auto publish8B = std::make_shared<Decode_8B>(node);
    #endif
    #ifdef PROTOCOL_BDDB0B
    auto publish0B = std::make_shared<Decode_0B>(node);
    #endif
    #ifdef PROTOCOL_BDDB10
    auto publish10 = std::make_shared<Decode_10>(node);
    #endif
    #ifdef PROTOCOL_BDDB1B
    auto publish1B = std::make_shared<Decode_1B>(node);
    #endif

    #ifdef ROS2_PLATFORM
    //循环节点
    rclcpp::spin(node);
    rclcpp::shutdown();
    #endif

    #ifdef ROS1_PLATFORM
    ros::spin();
    ros::shutdown();
    #endif

    return 0;
}
