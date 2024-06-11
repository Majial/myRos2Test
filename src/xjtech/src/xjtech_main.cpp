#include "XjtechRos2.hpp"

#include <iostream>

using namespace std;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                          // ROS2 C++接口初始化
    rclcpp::spin(std::make_shared<XjtechRos2>());   // 创建ROS2节点对象并进行初始化
    rclcpp::shutdown();                                // 关闭ROS2 C++接口
    return 0;
}
