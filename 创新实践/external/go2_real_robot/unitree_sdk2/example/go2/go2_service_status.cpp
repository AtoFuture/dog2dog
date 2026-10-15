#include <iostream>
#include <string>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

int main(int argc, char** argv)
{
    const std::string network_interface = argc >= 2 ? argv[1] : "eth0";
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    unitree::robot::go2::RobotStateClient client;
    client.SetTimeout(5.0f);
    client.Init();

    std::vector<unitree::robot::go2::ServiceState> services;
    const int32_t ret = client.ServiceList(services);
    std::cout << "ServiceList ret=" << ret << " count=" << services.size() << std::endl;
    if (ret != 0) {
        return 1;
    }
    for (const auto& service : services) {
        std::cout << "name=" << service.name
                  << " status=" << service.status
                  << " protect=" << service.protect
                  << std::endl;
    }
    return 0;
}
