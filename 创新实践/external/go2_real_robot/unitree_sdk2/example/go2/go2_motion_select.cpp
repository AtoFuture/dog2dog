#include <iostream>
#include <string>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

int main(int argc, char** argv)
{
    const std::string network_interface = argc >= 2 ? argv[1] : "eth0";
    const std::string mode = argc >= 3 ? argv[2] : "normal";
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    unitree::robot::b2::MotionSwitcherClient client;
    client.SetTimeout(5.0f);
    client.Init();
    const int32_t ret = client.SelectMode(mode);
    std::cout << "SelectMode mode=" << mode << " ret=" << ret << std::endl;

    std::string form;
    std::string active_mode;
    const int32_t check_ret = client.CheckMode(form, active_mode);
    std::cout << "CheckMode ret=" << check_ret
              << " form=" << form
              << " mode=" << active_mode
              << std::endl;
    return ret == 0 && check_ret == 0 && active_mode == mode ? 0 : 1;
}
