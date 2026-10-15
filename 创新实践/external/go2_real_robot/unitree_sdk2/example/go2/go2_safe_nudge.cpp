#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

using namespace unitree::robot;
using namespace unitree::robot::go2;

namespace
{

constexpr float kMinDistanceM = 0.05f;
constexpr float kMaxDistanceM = 0.20f;
constexpr float kSpeedMps = 0.25f;
constexpr int kControlPeriodMs = 20;

class StopGuard
{
public:
    explicit StopGuard(SportClient& client) : client_(client) {}

    ~StopGuard()
    {
        if (!stopped_) {
            client_.StopMove();
        }
    }

    int32_t Stop()
    {
        const int32_t ret = client_.StopMove();
        stopped_ = true;
        return ret;
    }

private:
    SportClient& client_;
    bool stopped_ = false;
};

}  // namespace

int main(int argc, const char** argv)
{
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0]
                  << " network_interface forward|backward distance_m" << std::endl;
        return 2;
    }

    const std::string network_interface = argv[1];
    const std::string direction = argv[2];
    const float distance_m = std::stof(argv[3]);
    if (direction != "forward" && direction != "backward") {
        std::cerr << "direction must be forward or backward" << std::endl;
        return 2;
    }
    if (!std::isfinite(distance_m) || distance_m < kMinDistanceM || distance_m > kMaxDistanceM) {
        std::cerr << "distance_m must be between " << kMinDistanceM
                  << " and " << kMaxDistanceM << std::endl;
        return 2;
    }

    ChannelFactory::Instance()->Init(0, network_interface);
    SportClient client;
    client.SetTimeout(3.0f);
    client.Init();
    StopGuard stop_guard(client);

    const int32_t stand_ret = client.StandUp();
    std::cout << "StandUp ret=" << stand_ret << std::endl;
    if (stand_ret != 0) {
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    const int32_t balance_ret = client.BalanceStand();
    std::cout << "BalanceStand ret=" << balance_ret << std::endl;
    if (balance_ret != 0) {
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    const float velocity_mps = direction == "forward" ? kSpeedMps : -kSpeedMps;
    const auto command_duration = std::chrono::duration<float>(distance_m / kSpeedMps);
    const auto start = std::chrono::steady_clock::now();
    int32_t move_ret = 0;
    int command_count = 0;
    while (std::chrono::steady_clock::now() - start < command_duration) {
        move_ret = client.Move(velocity_mps, 0.0f, 0.0f);
        if (move_ret != 0) {
            break;
        }
        ++command_count;
        std::this_thread::sleep_for(std::chrono::milliseconds(kControlPeriodMs));
    }

    const int32_t stop_ret = stop_guard.Stop();
    const float elapsed_s = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - start).count();
    std::cout << "Move ret=" << move_ret
              << " StopMove ret=" << stop_ret
              << " commands=" << command_count
              << " elapsed_s=" << elapsed_s
              << " commanded_distance_m=" << std::min(distance_m, elapsed_s * kSpeedMps)
              << std::endl;
    return move_ret == 0 && stop_ret == 0 ? 0 : 1;
}
