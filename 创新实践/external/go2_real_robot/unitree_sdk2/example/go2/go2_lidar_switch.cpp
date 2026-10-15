/**********************************************************************
 GO2 激光雷达开关控制
 功能：通过 DDS 话题 rt/utlidar/switch 发送 ON/OFF 指令
***********************************************************************/

#include <iostream>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/common/time/time_tool.hpp>

#define TOPIC_SWITCH "rt/utlidar/switch"

using namespace unitree::robot;
using namespace unitree::common;

int main(int argc, const char** argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface <ON|OFF>" << std::endl;
        std::cout << "  ON  - 打开激光雷达" << std::endl;
        std::cout << "  OFF - 关闭激光雷达" << std::endl;
        exit(-1);
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    std::string cmd = argv[2];
    if (cmd != "ON" && cmd != "OFF")
    {
        std::cout << "错误：参数必须是 ON 或 OFF" << std::endl;
        exit(-1);
    }

    ChannelPublisherPtr<std_msgs::msg::dds_::String_> publisher;
    publisher.reset(new ChannelPublisher<std_msgs::msg::dds_::String_>(TOPIC_SWITCH));
    publisher->InitChannel();

    std_msgs::msg::dds_::String_ switch_msg;
    switch_msg.data() = cmd;

    // 连续发送几次确保指令被接收
    for (int i = 0; i < 5; i++)
    {
        publisher->Write(switch_msg);
        std::cout << "已发送指令: " << cmd << " (" << (i+1) << "/5)" << std::endl;
        sleep(1);
    }

    std::cout << "\n激光雷达" << (cmd == "ON" ? "已打开" : "已关闭") << std::endl;

    return 0;
}
