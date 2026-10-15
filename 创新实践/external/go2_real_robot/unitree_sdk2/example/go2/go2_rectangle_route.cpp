/**********************************************************************
 GO2 rectangle route test
 Flow:
   StandUp -> BalanceStand
   forward 8m -> right 90deg -> forward 3m -> right 90deg
   forward 8m -> right 90deg -> forward 3m
   StandDown
***********************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;
using namespace unitree::robot::go2;

class GO2RectangleRoute
{
public:
    GO2RectangleRoute()
    {
        sport_client.SetTimeout(10.0f);
        sport_client.Init();

        suber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&GO2RectangleRoute::StateHandler, this, std::placeholders::_1), 1);
    }

    void Run()
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "\n========== GO2 rectangle route ==========" << std::endl;
        PrintState("Initial");

        CallAndWait("StandUp", [&]() { return sport_client.StandUp(); }, 1500);
        CallAndWait("BalanceStand", [&]() { return sport_client.BalanceStand(); }, 800);

        float route_yaw = state.imu_state().rpy()[2];
        MoveStraight("Side 1: forward 8m", 8.0f, route_yaw);

        route_yaw = TurnRight90("Turn 1: right 90deg");
        MoveStraight("Side 2: forward 3m", 3.0f, route_yaw);

        route_yaw = TurnRight90("Turn 2: right 90deg");
        MoveStraight("Side 3: forward 8m", 8.0f, route_yaw);

        route_yaw = TurnRight90("Turn 3: right 90deg");
        MoveStraight("Side 4: forward 3m", 3.0f, route_yaw);

        CallAndWait("StandDown", [&]() { return sport_client.StandDown(); }, 1500);
        PrintState("Final");
        std::cout << "========== rectangle route finished ==========\n" << std::endl;
    }

private:
    static float NormalizeAngle(float angle)
    {
        constexpr float pi = 3.14159265358979323846f;
        while (angle > pi)
        {
            angle -= 2.0f * pi;
        }
        while (angle < -pi)
        {
            angle += 2.0f * pi;
        }
        return angle;
    }

    template <typename Func>
    void CallAndWait(const std::string& name, Func func, int wait_ms)
    {
        std::cout << "\n>> " << name << std::endl;
        int32_t ret = func();
        std::cout << "   ret: " << ret << (ret == 0 ? " (ok)" : " (failed)") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }

    void MoveStraight(const std::string& label, float target_distance, float target_yaw)
    {
        const float start_x = state.position()[0];
        const float start_y = state.position()[1];
        const float dir_x = std::cos(target_yaw);
        const float dir_y = std::sin(target_yaw);
        const float tolerance = 0.12f;
        const float cruise_speed = 0.50f;
        const float approach_speed = 0.35f;
        const float approach_distance = 0.80f;
        const float yaw_kp = 0.9f;
        const float max_yaw_speed = 0.35f;
        const int loop_ms = 10;
        const float max_time_sec = target_distance / 0.30f + 8.0f;

        auto progress_from_start = [&]() {
            const float dx = state.position()[0] - start_x;
            const float dy = state.position()[1] - start_y;
            return dx * dir_x + dy * dir_y;
        };

        std::cout << "\n>> " << label << std::endl;
        std::cout << "   target distance: " << target_distance << " m" << std::endl;
        std::cout << "   target yaw: " << target_yaw << " rad" << std::endl;

        int elapsed_ms = 0;
        int32_t last_ret = 0;
        float progress = 0.0f;

        while (elapsed_ms < static_cast<int>(max_time_sec * 1000))
        {
            progress = progress_from_start();
            const float remaining = target_distance - progress;
            if (remaining <= tolerance)
            {
                break;
            }

            const float speed = (remaining < approach_distance) ? approach_speed : cruise_speed;
            const float yaw_error = NormalizeAngle(target_yaw - state.imu_state().rpy()[2]);
            const float vyaw = std::clamp(yaw_kp * yaw_error, -max_yaw_speed, max_yaw_speed);

            last_ret = sport_client.Move(speed, 0.0f, vyaw);
            std::this_thread::sleep_for(std::chrono::milliseconds(loop_ms));
            elapsed_ms += loop_ms;

            if (elapsed_ms % 1000 == 0)
            {
                std::cout << "   t=" << elapsed_ms / 1000.0f
                          << "s, progress=" << progress
                          << "m, remaining=" << std::max(0.0f, remaining)
                          << "m, vx=" << speed
                          << "m/s, yaw_error=" << yaw_error
                          << "rad, vyaw=" << vyaw << "rad/s" << std::endl;
            }
        }

        std::cout << "   Move ret: " << last_ret << std::endl;
        int32_t stop_ret = sport_client.StopMove();
        std::cout << "   StopMove ret: " << stop_ret << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(700));

        const float final_progress = progress_from_start();
        std::cout << "   final progress: " << final_progress
                  << " m, error: " << target_distance - final_progress << " m" << std::endl;
    }

    float TurnRight90(const std::string& label)
    {
        constexpr float pi = 3.14159265358979323846f;
        const float start_yaw = state.imu_state().rpy()[2];
        const float target_yaw = NormalizeAngle(start_yaw - pi / 2.0f);
        const float tolerance = 0.045f;
        const int loop_ms = 10;
        const int max_time_ms = 7000;

        std::cout << "\n>> " << label << std::endl;
        std::cout << "   start yaw: " << start_yaw << " rad, target yaw: " << target_yaw << " rad" << std::endl;

        int elapsed_ms = 0;
        int32_t last_ret = 0;
        float yaw_error = NormalizeAngle(target_yaw - state.imu_state().rpy()[2]);

        while (elapsed_ms < max_time_ms)
        {
            yaw_error = NormalizeAngle(target_yaw - state.imu_state().rpy()[2]);
            if (std::fabs(yaw_error) <= tolerance)
            {
                break;
            }

            const float abs_error = std::fabs(yaw_error);
            const float turn_speed = (abs_error > 0.30f) ? 0.65f : 0.32f;
            const float vyaw = std::copysign(turn_speed, yaw_error);

            last_ret = sport_client.Move(0.0f, 0.0f, vyaw);
            std::this_thread::sleep_for(std::chrono::milliseconds(loop_ms));
            elapsed_ms += loop_ms;

            if (elapsed_ms % 500 == 0)
            {
                std::cout << "   t=" << elapsed_ms / 1000.0f
                          << "s, yaw_error=" << yaw_error
                          << "rad, vyaw=" << vyaw << "rad/s" << std::endl;
            }
        }

        std::cout << "   Move ret: " << last_ret << std::endl;
        int32_t stop_ret = sport_client.StopMove();
        std::cout << "   StopMove ret: " << stop_ret << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(700));

        const float final_yaw = state.imu_state().rpy()[2];
        const float final_error = NormalizeAngle(target_yaw - final_yaw);
        std::cout << "   final yaw: " << final_yaw
                  << " rad, target error: " << final_error << " rad" << std::endl;
        return target_yaw;
    }

    void PrintState(const std::string& title)
    {
        std::cout << "\n[" << title << "]" << std::endl;
        std::cout << "  position: x=" << state.position()[0]
                  << ", y=" << state.position()[1]
                  << ", z=" << state.position()[2] << std::endl;
        std::cout << "  rpy: roll=" << state.imu_state().rpy()[0]
                  << ", pitch=" << state.imu_state().rpy()[1]
                  << ", yaw=" << state.imu_state().rpy()[2] << std::endl;
    }

    void StateHandler(const void* message)
    {
        state = *(unitree_go::msg::dds_::SportModeState_*)message;
    }

    unitree_go::msg::dds_::SportModeState_ state;
    SportClient sport_client;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
};

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0" << std::endl;
        return -1;
    }

    std::cout << "network interface: " << argv[1] << std::endl;
    ChannelFactory::Instance()->Init(0, argv[1]);

    GO2RectangleRoute route;
    route.Run();
    return 0;
}
