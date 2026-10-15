/**********************************************************************
 GO2 distance closed-loop test
 Flow: stand up -> balance stand -> forward 0.30m -> back 0.30m -> stand down
***********************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;
using namespace unitree::robot::go2;

class GO2DistanceControl
{
public:
    GO2DistanceControl()
    {
        sport_client.SetTimeout(10.0f);
        sport_client.Init();

        suber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&GO2DistanceControl::StateHandler, this, std::placeholders::_1), 1);
    }

    void Run(float target_distance_m)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "\n========== GO2距离闭环运动测试 ==========" << std::endl;
        PrintState("初始状态");

        CallAndWait("站立 StandUp", [&]() { return sport_client.StandUp(); }, 1500);
        CallAndWait("平衡站立 BalanceStand", [&]() { return sport_client.BalanceStand(); }, 800);

        const float route_yaw = state.imu_state().rpy()[2];
        std::cout << "\n路线锁定 yaw: " << route_yaw << " rad" << std::endl;

        MoveDistance("前进", target_distance_m, 0.26f, route_yaw);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        MoveDistance("后退", target_distance_m, -0.26f, route_yaw);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        CallAndWait("趴下 StandDown", [&]() { return sport_client.StandDown(); }, 1500);
        PrintState("结束状态");
        std::cout << "========== 测试结束 ==========\n" << std::endl;
    }

private:
    template <typename Func>
    void CallAndWait(const std::string& name, Func func, int wait_ms)
    {
        std::cout << "\n>> " << name << std::endl;
        int32_t ret = func();
        std::cout << "   返回码: " << ret << (ret == 0 ? " (成功)" : " (失败)") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }

    float NormalizeAngle(float angle)
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

    void MoveDistance(const std::string& label, float target_distance_m, float vx, float target_yaw)
    {
        const float start_x = state.position()[0];
        const float start_y = state.position()[1];
        const float dir_x = std::cos(target_yaw);
        const float dir_y = std::sin(target_yaw);
        const float abs_target = std::fabs(target_distance_m);
        const float tolerance = std::max(0.003f, abs_target * 0.01f);
        const float max_time_sec = std::max(8.0f, abs_target / 0.35f + 6.0f);
        const int loop_ms = 10;
        const float direction_sign = (vx >= 0.0f) ? 1.0f : -1.0f;
        const float fast_speed = 0.60f;
        const float medium_speed = 0.45f;
        const float slow_speed = 0.12f;
        const float medium_phase_start = abs_target * 0.60f;
        const float slow_phase_start = abs_target * 0.90f;
        const float yaw_kp = 0.8f;
        const float max_yaw_speed = 0.35f;

        std::cout << "\n>> " << label << "闭环控制" << std::endl;
        std::cout << "   目标距离: " << abs_target << " m" << std::endl;
        std::cout << "   允许误差: " << tolerance << " m" << std::endl;
        std::cout << "   三段式: 前60%速度=" << fast_speed
                  << " m/s, 中间30%速度=" << medium_speed
                  << " m/s, 最后10%速度=" << slow_speed << " m/s" << std::endl;
        std::cout << "   方向: " << (direction_sign > 0.0f ? "前进" : "后退") << std::endl;
        std::cout << "   起点: x=" << start_x << ", y=" << start_y
                  << ", 当前yaw=" << state.imu_state().rpy()[2]
                  << ", 目标yaw=" << target_yaw << std::endl;

        auto progress_from_start = [&]() {
            const float dx = state.position()[0] - start_x;
            const float dy = state.position()[1] - start_y;
            const float signed_progress = dx * dir_x + dy * dir_y;
            return direction_sign * signed_progress;
        };

        int32_t last_ret = 0;
        float progress = 0.0f;
        int elapsed_ms = 0;
        while (elapsed_ms < static_cast<int>(max_time_sec * 1000))
        {
            progress = progress_from_start();
            const float remaining = abs_target - progress;
            if (remaining <= tolerance)
            {
                break;
            }

            float speed = fast_speed;
            const char* phase = "快速";
            if (progress >= slow_phase_start)
            {
                speed = slow_speed;
                phase = "低速";
            }
            else if (progress >= medium_phase_start)
            {
                speed = medium_speed;
                phase = "中速";
            }
            const float yaw_error = NormalizeAngle(target_yaw - state.imu_state().rpy()[2]);
            const float vyaw = std::clamp(yaw_kp * yaw_error, -max_yaw_speed, max_yaw_speed);
            last_ret = sport_client.Move(std::copysign(speed, direction_sign), 0.0f, vyaw);
            std::this_thread::sleep_for(std::chrono::milliseconds(loop_ms));
            elapsed_ms += loop_ms;

            if (elapsed_ms % 500 == 0)
            {
                std::cout << "   t=" << elapsed_ms / 1000.0f
                          << "s, 已走=" << progress
                          << "m, 剩余=" << std::max(0.0f, remaining)
                          << "m, 阶段=" << phase
                          << ", 速度=" << std::copysign(speed, direction_sign)
                          << "m/s, yaw误差=" << yaw_error
                          << "rad, vyaw=" << vyaw << "rad/s" << std::endl;
            }
        }

        std::cout << "   Move最后返回码: " << last_ret << std::endl;
        int32_t stop_ret = sport_client.StopMove();
        std::cout << "   StopMove返回码: " << stop_ret << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(700));

        const float end_x = state.position()[0];
        const float end_y = state.position()[1];
        const float dx = end_x - start_x;
        const float dy = end_y - start_y;
        const float signed_progress = dx * dir_x + dy * dir_y;
        const float final_progress = direction_sign * signed_progress;
        const float final_error = abs_target - final_progress;
        std::cout << "   终点: x=" << end_x << ", y=" << end_y << std::endl;
        std::cout << "   沿初始朝向实际位移: " << final_progress << " m" << std::endl;
        std::cout << "   最终误差: " << final_error << " m ("
                  << (std::fabs(final_error) / abs_target * 100.0f) << "%)" << std::endl;
        std::cout << "   世界坐标变化: dX=" << dx << " m, dY=" << dy << " m" << std::endl;
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
        std::cout << "  velocity: x=" << state.velocity()[0]
                  << ", y=" << state.velocity()[1]
                  << ", z=" << state.velocity()[2] << std::endl;
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
        std::cout << "Usage: " << argv[0] << " networkInterface [distance_m]" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0 0.30" << std::endl;
        return -1;
    }

    float distance = 0.30f;
    if (argc >= 3)
    {
        distance = std::stof(argv[2]);
    }
    distance = std::clamp(distance, 0.05f, 10.0f);

    std::cout << "网络接口: " << argv[1] << std::endl;
    std::cout << "闭环目标距离: " << distance << " m" << std::endl;

    ChannelFactory::Instance()->Init(0, argv[1]);
    GO2DistanceControl control;
    control.Run(distance);
    return 0;
}
