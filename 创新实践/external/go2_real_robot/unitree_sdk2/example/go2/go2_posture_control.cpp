/**********************************************************************
 GO2 posture and status control.
 Actions:
   status | damp | balance_stand | stand_up | stand_down | recovery_stand | stop_move
***********************************************************************/

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;
using namespace unitree::robot::go2;

namespace
{

constexpr float kLowPostureZ = 0.12f;
constexpr float kStandingZ = 0.20f;

enum class ExpectedState
{
    AnyFreshState,
    DampOrLow,
    LowPosture,
    Standing,
    Balance,
    Recovery
};

struct StateSnapshot
{
    bool received = false;
    uint64_t seq = 0;
    int mode = -1;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

void PrintUsage(const char* program)
{
    std::cout << "Usage: " << program << " networkInterface action [status_seconds]\n"
              << "Actions:\n"
              << "  status\n"
              << "  damp\n"
              << "  balance_stand\n"
              << "  stand_up\n"
              << "  stand_down\n"
              << "  recovery_stand\n"
              << "  stop_move\n";
}

class GO2PostureControl
{
public:
    GO2PostureControl()
    {
        sport_client.SetTimeout(10.0f);
        sport_client.Init();

        state_client.SetTimeout(10.0f);
        state_client.Init();

        subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        subscriber->InitChannel(std::bind(&GO2PostureControl::StateHandler, this, std::placeholders::_1), 1);
    }

    ~GO2PostureControl()
    {
        subscriber.reset();
    }

    int RunAction(const std::string& action, int status_seconds)
    {
        if (action == "status") {
            return PrintStatus(status_seconds);
        }
        if (action == "damp") {
            return CallSport("Damp", [&]() { return sport_client.Damp(); }, ExpectedState::DampOrLow, 2500);
        }
        if (action == "balance_stand") {
            return CallSport("BalanceStand", [&]() { return sport_client.BalanceStand(); }, ExpectedState::Balance, 4000);
        }
        if (action == "stand_up") {
            return CallSport("StandUp", [&]() { return sport_client.StandUp(); }, ExpectedState::Standing, 4000);
        }
        if (action == "stand_down") {
            return CallSport("StandDown", [&]() { return sport_client.StandDown(); }, ExpectedState::LowPosture, 4000);
        }
        if (action == "recovery_stand") {
            return CallSport("RecoveryStand", [&]() { return sport_client.RecoveryStand(); }, ExpectedState::Recovery, 6000);
        }
        if (action == "stop_move") {
            return CallSport("StopMove", [&]() { return sport_client.StopMove(); }, ExpectedState::AnyFreshState, 1500);
        }

        std::cout << "Unknown action: " << action << std::endl;
        return 2;
    }

private:
    template <typename Func>
    int CallSport(const std::string& name, Func func, ExpectedState expected, int timeout_ms)
    {
        if (!WaitForState(2000)) {
            std::cout << "未收到 rt/sportmodestate，无法确认当前状态。" << std::endl;
            return 1;
        }

        const StateSnapshot before = Snapshot();
        std::cout << "action:" << name << std::endl;
        int32_t ret = func();
        std::cout << "ret:" << ret << (ret == 0 ? " ok" : " failed") << std::endl;
        const bool reached = ret == 0 && WaitForExpected(expected, before.seq, timeout_ms);
        const StateSnapshot after = Snapshot();
        PrintSportState(after);
        std::cout << "当前状态：" << StatusText(after) << std::endl;
        if (!reached) {
            std::cout << "结果：动作未达到目标状态，已判定失败。" << std::endl;
        }
        return reached ? 0 : 1;
    }

    int PrintStatus(int status_seconds)
    {
        status_seconds = std::max(1, std::min(status_seconds, 10));
        if (!WaitForState(2000)) {
            std::cout << "当前状态：状态未更新（未收到 rt/sportmodestate）" << std::endl;
            return 1;
        }
        const uint64_t before_seq = Snapshot().seq;
        WaitForNextState(before_seq, status_seconds * 1000);
        QueryServiceStateForLog();
        std::cout << "当前状态：" << StatusText(Snapshot()) << std::endl;
        return 0;
    }

    std::string StatusText(const StateSnapshot& snapshot) const
    {
        if (!snapshot.received) {
            return "状态未更新";
        }

        switch (snapshot.mode) {
        case 0:
            return "阻尼";
        case 1:
        case 5:
            return "坐下";
        case 2:
        case 6:
            return "站立";
        case 3:
        case 4:
        case 9:
        case 13:
            return "平衡站立";
        case 12:
            return "恢复站立";
        case 7:
            if (snapshot.z < kLowPostureZ) {
                return "坐下/低姿态";
            }
            return "阻尼/过渡";
        default:
            if (snapshot.z >= kStandingZ) {
                return "平衡站立";
            }
            return "坐下";
        }
    }

    bool MatchesExpected(const StateSnapshot& snapshot, ExpectedState expected) const
    {
        if (!snapshot.received) {
            return false;
        }

        switch (expected) {
        case ExpectedState::AnyFreshState:
            return true;
        case ExpectedState::DampOrLow:
            return snapshot.mode == 0 || (snapshot.mode == 7 && snapshot.z < kLowPostureZ) || snapshot.z < kLowPostureZ;
        case ExpectedState::LowPosture:
            return snapshot.mode == 1 || snapshot.mode == 5 || snapshot.z < kLowPostureZ;
        case ExpectedState::Standing:
            return snapshot.mode == 2 || snapshot.mode == 6 || snapshot.z >= kStandingZ;
        case ExpectedState::Balance:
            return snapshot.mode == 3 || snapshot.mode == 4 || snapshot.mode == 9 || snapshot.mode == 13;
        case ExpectedState::Recovery:
            return snapshot.mode == 12 || snapshot.mode == 2 || snapshot.mode == 6 || snapshot.z >= kStandingZ;
        }
        return false;
    }

    bool WaitForExpected(ExpectedState expected, uint64_t min_seq, int timeout_ms)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            StateSnapshot snapshot = Snapshot();
            if (snapshot.seq > min_seq && MatchesExpected(snapshot, expected)) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return MatchesExpected(Snapshot(), expected);
    }

    bool WaitForState(int timeout_ms)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            if (Snapshot().received) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return Snapshot().received;
    }

    bool WaitForNextState(uint64_t min_seq, int timeout_ms)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            if (Snapshot().seq > min_seq) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false;
    }

    StateSnapshot Snapshot() const
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        StateSnapshot snapshot;
        snapshot.received = state_received;
        snapshot.seq = state_seq;
        snapshot.mode = static_cast<int>(state.mode());
        snapshot.x = state.position()[0];
        snapshot.y = state.position()[1];
        snapshot.z = state.position()[2];
        snapshot.roll = state.imu_state().rpy()[0];
        snapshot.pitch = state.imu_state().rpy()[1];
        snapshot.yaw = state.imu_state().rpy()[2];
        return snapshot;
    }

    void QueryServiceStateForLog()
    {
        std::vector<ServiceState> service_state_list;
        state_client.ServiceList(service_state_list);
    }

    void PrintSportState(const StateSnapshot& snapshot)
    {
        std::cout << "sport_state:"
                  << " x=" << snapshot.x
                  << " y=" << snapshot.y
                  << " z=" << snapshot.z
                  << " mode=" << snapshot.mode
                  << " roll=" << snapshot.roll
                  << " pitch=" << snapshot.pitch
                  << " yaw=" << snapshot.yaw
                  << std::endl;
    }

    void StateHandler(const void* message)
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        state = *(unitree_go::msg::dds_::SportModeState_*)message;
        state_received = true;
        ++state_seq;
    }

    unitree_go::msg::dds_::SportModeState_ state;
    bool state_received = false;
    uint64_t state_seq = 0;
    mutable std::mutex state_mutex;
    SportClient sport_client;
    RobotStateClient state_client;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> subscriber;
};

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3) {
        PrintUsage(argv[0]);
        return 2;
    }

    const std::string network_interface = argv[1];
    const std::string action = argv[2];
    int status_seconds = 3;
    if (argc >= 4) {
        status_seconds = std::stoi(argv[3]);
    }

    ChannelFactory::Instance()->Init(0, network_interface);
    GO2PostureControl control;
    return control.RunAction(action, status_seconds);
}
