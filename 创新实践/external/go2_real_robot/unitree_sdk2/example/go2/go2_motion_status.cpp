#include <iostream>
#include <string>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

int main(int argc, char** argv)
{
    const std::string network_interface = argc >= 2 ? argv[1] : "eth0";
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    unitree::robot::b2::MotionSwitcherClient client;
    client.SetTimeout(5.0f);
    client.Init();

    std::string form;
    std::string mode;
    int32_t ret = client.CheckMode(form, mode);
    std::cout << "CheckMode ret=" << ret << " form=" << form << " mode=" << mode << std::endl;

    bool silent = false;
    ret = client.GetSilent(silent);
    std::cout << "GetSilent ret=" << ret << " silent=" << (silent ? "true" : "false") << std::endl;
    return ret == 0 ? 0 : 1;
}
