/**********************************************************************
 GO2 速度控制测试程序 - 室内安全版本
 功能：速度控制、前进/后退/转向、停止/急停
 注意：室内空间有限，速度值已设置为安全范围
***********************************************************************/

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

class GO2SpeedControl
{
public:
    GO2SpeedControl()
    {
        sport_client.SetTimeout(10.0f);
        sport_client.Init();

        state_client.SetTimeout(10.0f);
        state_client.Init();

        suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&GO2SpeedControl::StateHandler, this, std::placeholders::_1), 1);
    }

    // 打印机器人状态
    void PrintState()
    {
        std::cout << "\n========== 机器人状态 ==========" << std::endl;
        std::cout << "位置 - X: " << state.position()[0] 
                  << ", Y: " << state.position()[1] 
                  << ", Z: " << state.position()[2] << std::endl;
        std::cout << "姿态 - Roll: " << state.imu_state().rpy()[0] 
                  << ", Pitch: " << state.imu_state().rpy()[1] 
                  << ", Yaw: " << state.imu_state().rpy()[2] << std::endl;
        std::cout << "速度 - X: " << state.velocity()[0] 
                  << ", Y: " << state.velocity()[1] 
                  << ", Z: " << state.velocity()[2] << std::endl;
        std::cout << "================================\n" << std::endl;
    }

    // 查询服务状态
    void QueryServiceState()
    {
        std::cout << "\n========== 服务状态 ==========" << std::endl;
        std::vector<ServiceState> serviceStateList;
        int32_t ret = state_client.ServiceList(serviceStateList);
        
        if (ret == 0)
        {
            std::cout << "服务数量: " << serviceStateList.size() << std::endl;
            for (size_t i = 0; i < serviceStateList.size(); i++)
            {
                const ServiceState& service = serviceStateList[i];
                std::cout << "  [" << i << "] " << service.name 
                          << " - 状态: " << service.status 
                          << ", 保护: " << service.protect << std::endl;
            }
        }
        else
        {
            std::cout << "获取服务状态失败，错误码: " << ret << std::endl;
        }
        std::cout << "================================\n" << std::endl;
    }

    // 站立
    void StandUp()
    {
        std::cout << "\n>> 执行：站立" << std::endl;
        int32_t ret = sport_client.StandUp();
        std::cout << "   返回码: " << ret << (ret == 0 ? " (成功)" : " (失败)") << "\n" << std::endl;
    }

    // 趴下
    void StandDown()
    {
        std::cout << "\n>> 执行：趴下" << std::endl;
        int32_t ret = sport_client.StandDown();
        std::cout << "   返回码: " << ret << (ret == 0 ? " (成功)" : " (失败)") << "\n" << std::endl;
    }

    // 平衡站立
    void BalanceStand()
    {
        std::cout << "\n>> 执行：平衡站立" << std::endl;
        int32_t ret = sport_client.BalanceStand();
        std::cout << "   返回码: " << ret << (ret == 0 ? " (成功)" : " (失败)") << "\n" << std::endl;
    }

    // 阻尼模式（急停）
    void Damp()
    {
        std::cout << "\n>> 执行：阻尼模式（急停）" << std::endl;
        int32_t ret = sport_client.Damp();
        std::cout << "   返回码: " << ret << (ret == 0 ? " (成功)" : " (失败)") << "\n" << std::endl;
    }

    // 恢复站立
    void RecoveryStand()
    {
        std::cout << "\n>> 执行：恢复站立" << std::endl;
        int32_t ret = sport_client.RecoveryStand();
        std::cout << "   返回码: " << ret << (ret == 0 ? " (成功)" : " (失败)") << "\n" << std::endl;
    }

    // 停止运动
    void StopMove()
    {
        std::cout << "\n>> 执行：停止运动" << std::endl;
        int32_t ret = sport_client.StopMove();
        std::cout << "   返回码: " << ret << (ret == 0 ? " (成功)" : " (失败)") << "\n" << std::endl;
    }

    // 速度控制 - 前进/后退/左右/转向
    // vx: 前后速度 (-1.0 ~ 1.0 m/s)，正值前进，负值后退
    // vy: 左右速度 (-0.5 ~ 0.5 m/s)，正值左移，负值右移
    // vyaw: 旋转速度 (-1.0 ~ 1.0 rad/s)，正值左转，负值右转
    void Move(float vx, float vy, float vyaw, float duration_sec = 1.0f)
    {
        // 记录初始状态
        float init_pos_x = state.position()[0];
        float init_pos_y = state.position()[1];
        float init_pos_z = state.position()[2];
        float init_vel_x = state.velocity()[0];
        float init_vel_y = state.velocity()[1];
        float init_vel_z = state.velocity()[2];
        float init_roll = state.imu_state().rpy()[0];
        float init_pitch = state.imu_state().rpy()[1];
        float init_yaw = state.imu_state().rpy()[2];

        // 打印移动前的完整状态
        std::cout << "\n========== 移动前状态 ==========" << std::endl;
        std::cout << "[DEBUG] 位置 - X: " << init_pos_x << ", Y: " << init_pos_y << ", Z: " << init_pos_z << std::endl;
        std::cout << "[DEBUG] 速度 - X: " << init_vel_x << ", Y: " << init_vel_y << ", Z: " << init_vel_z << std::endl;
        std::cout << "[DEBUG] 姿态 - Roll: " << init_roll << ", Pitch: " << init_pitch << ", Yaw: " << init_yaw << " rad" << std::endl;
        std::cout << "================================\n" << std::endl;

        // 先切换到平衡模式，确保可以移动
        std::cout << "\n>> 切换到平衡模式..." << std::endl;
        int32_t balance_ret = sport_client.BalanceStand();
        std::cout << "   [DEBUG] BalanceStand 返回码: " << balance_ret << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "\n>> 执行：速度控制" << std::endl;
        std::cout << "   [DEBUG] 发送参数: vx=" << vx << " m/s, vy=" << vy << " m/s, vyaw=" << vyaw << " rad/s" << std::endl;
        std::cout << "   [DEBUG] 持续时间: " << duration_sec << " 秒" << std::endl;
        std::cout << "   [DEBUG] 预期距离: " << (vx * duration_sec) << " m (前后), " << (vy * duration_sec) << " m (左右)" << std::endl;
        std::cout << "   [DEBUG] 预期旋转: " << (vyaw * duration_sec) << " rad (" << (vyaw * duration_sec * 180.0 / 3.14159) << " 度)" << std::endl;
        
        // 关键：Move 指令需要持续高频发送（每 5ms 一次）
        std::cout << "\n   [DEBUG] 开始持续发送 Move 指令..." << std::endl;
        int total_ms = static_cast<int>(duration_sec * 1000);
        int elapsed = 0;
        int32_t ret = 0;
        
        // 存储每秒的状态快照
        struct StateSnapshot {
            int second;
            float pos_x, pos_y, pos_z;
            float vel_x, vel_y, vel_z;
            float roll, pitch, yaw;
        };
        std::vector<StateSnapshot> snapshots;
        
        while (elapsed < total_ms) {
            ret = sport_client.Move(vx, vy, vyaw);
            elapsed += 5;
            
            // 每秒记录一次完整状态
            if (elapsed % 1000 < 5) {
                StateSnapshot snap;
                snap.second = elapsed / 1000;
                snap.pos_x = state.position()[0];
                snap.pos_y = state.position()[1];
                snap.pos_z = state.position()[2];
                snap.vel_x = state.velocity()[0];
                snap.vel_y = state.velocity()[1];
                snap.vel_z = state.velocity()[2];
                snap.roll = state.imu_state().rpy()[0];
                snap.pitch = state.imu_state().rpy()[1];
                snap.yaw = state.imu_state().rpy()[2];
                snapshots.push_back(snap);
                
                std::cout << "   [DEBUG] 第 " << snap.second << " 秒:" << std::endl;
                std::cout << "     位置 - X: " << snap.pos_x << ", Y: " << snap.pos_y << ", Z: " << snap.pos_z << std::endl;
                std::cout << "     速度 - X: " << snap.vel_x << ", Y: " << snap.vel_y << ", Z: " << snap.vel_z << std::endl;
                std::cout << "     姿态 - Roll: " << snap.roll << ", Pitch: " << snap.pitch << ", Yaw: " << snap.yaw << " rad" << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
        std::cout << "   [DEBUG] Move 最后返回码: " << ret << std::endl;
        
        std::cout << "\n>> 发送停止指令..." << std::endl;
        int32_t stop_ret = sport_client.StopMove();
        std::cout << "   [DEBUG] StopMove 返回码: " << stop_ret << std::endl;
        
        // 打印停止后的完整状态
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        float final_pos_x = state.position()[0];
        float final_pos_y = state.position()[1];
        float final_pos_z = state.position()[2];
        float final_vel_x = state.velocity()[0];
        float final_vel_y = state.velocity()[1];
        float final_vel_z = state.velocity()[2];
        float final_roll = state.imu_state().rpy()[0];
        float final_pitch = state.imu_state().rpy()[1];
        float final_yaw = state.imu_state().rpy()[2];
        
        std::cout << "\n========== 停止后状态 ==========" << std::endl;
        std::cout << "[DEBUG] 位置 - X: " << final_pos_x << ", Y: " << final_pos_y << ", Z: " << final_pos_z << std::endl;
        std::cout << "[DEBUG] 速度 - X: " << final_vel_x << ", Y: " << final_vel_y << ", Z: " << final_vel_z << std::endl;
        std::cout << "[DEBUG] 姿态 - Roll: " << final_roll << ", Pitch: " << final_pitch << ", Yaw: " << final_yaw << " rad" << std::endl;
        
        // 计算变化量
        std::cout << "\n========== 变化量汇总 ==========" << std::endl;
        std::cout << "[DEBUG] 位置变化 - dX: " << (final_pos_x - init_pos_x) << " m" << std::endl;
        std::cout << "[DEBUG] 位置变化 - dY: " << (final_pos_y - init_pos_y) << " m" << std::endl;
        std::cout << "[DEBUG] 位置变化 - dZ: " << (final_pos_z - init_pos_z) << " m" << std::endl;
        std::cout << "[DEBUG] 姿态变化 - dRoll: " << (final_roll - init_roll) << " rad (" << ((final_roll - init_roll) * 180.0 / 3.14159) << " 度)" << std::endl;
        std::cout << "[DEBUG] 姿态变化 - dPitch: " << (final_pitch - init_pitch) << " rad (" << ((final_pitch - init_pitch) * 180.0 / 3.14159) << " 度)" << std::endl;
        std::cout << "[DEBUG] 姿态变化 - dYaw: " << (final_yaw - init_yaw) << " rad (" << ((final_yaw - init_yaw) * 180.0 / 3.14159) << " 度)" << std::endl;
        
        // 打印轨迹摘要
        std::cout << "\n========== 运动轨迹摘要 ==========" << std::endl;
        for (const auto& snap : snapshots) {
            std::cout << "  第 " << snap.second << " 秒: dX=" << (snap.pos_x - init_pos_x) 
                      << "m, dY=" << (snap.pos_y - init_pos_y)
                      << "m, dYaw=" << (snap.yaw - init_yaw) << "rad" << std::endl;
        }
        std::cout << "================================\n" << std::endl;
    }

    // 前进（室内安全距离）
    void MoveForward()
    {
        // 0.3 m/s * 0.4秒 = 预期约10cm
        Move(0.3f, 0.0f, 0.0f, 0.4f);
    }

    // 后退（室内安全距离）
    void MoveBackward()
    {
        // -0.3 m/s * 0.4秒 = 预期约10cm后退
        Move(-0.3f, 0.0f, 0.0f, 0.4f);
    }

    // 左移（室内安全距离）
    void MoveLeft()
    {
        // 0.3 m/s * 0.4秒 = 预期约10cm左移
        Move(0.0f, 0.3f, 0.0f, 0.4f);
    }

    // 右移（室内安全距离）
    void MoveRight()
    {
        // -0.3 m/s * 0.4秒 = 预期约10cm右移
        Move(0.0f, -0.3f, 0.0f, 0.4f);
    }

    // 左转
    void TurnLeft()
    {
        // 0.3 rad/s * 0.5秒 ≈ 0.15 rad ≈ 8.6度，适合室内小空间初测
        Move(0.0f, 0.0f, 0.3f, 0.5f);
    }

    // 右转
    void TurnRight()
    {
        // -0.3 rad/s * 0.5秒 ≈ -0.15 rad ≈ -8.6度，适合室内小空间初测
        Move(0.0f, 0.0f, -0.3f, 0.5f);
    }

    // 自定义速度控制
    void CustomMove()
    {
        float vx, vy, vyaw;
        std::cout << "\n>> 自定义速度控制" << std::endl;
        std::cout << "   请输入前后速度 vx (-1.0 ~ 1.0 m/s): ";
        std::cin >> vx;
        std::cout << "   请输入左右速度 vy (-0.5 ~ 0.5 m/s): ";
        std::cin >> vy;
        std::cout << "   请输入旋转速度 vyaw (-1.0 ~ 1.0 rad/s): ";
        std::cin >> vyaw;
        
        // 限制速度范围
        vx = std::max(-1.0f, std::min(1.0f, vx));
        vy = std::max(-0.5f, std::min(0.5f, vy));
        vyaw = std::max(-1.0f, std::min(1.0f, vyaw));
        
        Move(vx, vy, vyaw, 2.0f);
    }

    // 显示菜单
    void ShowMenu()
    {
        std::cout << "\n========== GO2 速度控制测试菜单 ==========" << std::endl;
        std::cout << "--- 基础功能 ---" << std::endl;
        std::cout << "1 - 查询机器人状态" << std::endl;
        std::cout << "2 - 查询服务状态" << std::endl;
        std::cout << "3 - 站立 (StandUp)" << std::endl;
        std::cout << "4 - 趴下 (StandDown)" << std::endl;
        std::cout << "5 - 平衡站立 (BalanceStand)" << std::endl;
        std::cout << "\n--- 速度控制 ---" << std::endl;
        std::cout << "6 - 前进 (约20cm)" << std::endl;
        std::cout << "7 - 后退 (约20cm)" << std::endl;
        std::cout << "8 - 左移 (约10cm)" << std::endl;
        std::cout << "9 - 右移 (约10cm)" << std::endl;
        std::cout << "a - 左转 (约9度)" << std::endl;
        std::cout << "b - 右转 (约9度)" << std::endl;
        std::cout << "c - 自定义速度控制" << std::endl;
        std::cout << "\n--- 停止/急停 ---" << std::endl;
        std::cout << "d - 停止运动 (StopMove)" << std::endl;
        std::cout << "e - 急停/阻尼模式 (Damp)" << std::endl;
        std::cout << "\nq - 退出程序" << std::endl;
        std::cout << "==========================================" << std::endl;
        std::cout << "请输入指令编号: ";
    }

    void Run()
    {
        sleep(1); // 等待1秒获取稳定状态
        
        std::cout << "\nGO2 速度控制测试程序已启动" << std::endl;
        std::cout << "安全提示：室内空间有限，速度已设置为安全范围" << std::endl;
        std::cout << "前后限制: 50cm, 左右限制: 20cm" << std::endl;

        while (true)
        {
            ShowMenu();
            std::string input;
            std::cin >> input;

            if (input == "q" || input == "Q")
            {
                std::cout << "退出程序" << std::endl;
                break;
            }

            if (input == "1")
            {
                PrintState();
            }
            else if (input == "2")
            {
                QueryServiceState();
            }
            else if (input == "3")
            {
                StandUp();
            }
            else if (input == "4")
            {
                StandDown();
            }
            else if (input == "5")
            {
                BalanceStand();
            }
            else if (input == "6")
            {
                MoveForward();
            }
            else if (input == "7")
            {
                MoveBackward();
            }
            else if (input == "8")
            {
                MoveLeft();
            }
            else if (input == "9")
            {
                MoveRight();
            }
            else if (input == "a" || input == "A")
            {
                TurnLeft();
            }
            else if (input == "b" || input == "B")
            {
                TurnRight();
            }
            else if (input == "c" || input == "C")
            {
                CustomMove();
            }
            else if (input == "d" || input == "D")
            {
                StopMove();
            }
            else if (input == "e" || input == "E")
            {
                Damp();
            }
            else
            {
                std::cout << "无效指令，请重新输入\n" << std::endl;
            }
        }
    }

private:
    void StateHandler(const void *message)
    {
        state = *(unitree_go::msg::dds_::SportModeState_ *)message;
    }

    unitree_go::msg::dds_::SportModeState_ state;
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::go2::RobotStateClient state_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
    
    // 初始位置记录
    float init_x = 0.0f;
    float init_y = 0.0f;
};

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "用法: " << argv[0] << " networkInterface" << std::endl;
        std::cout << "示例: " << argv[0] << " eth0" << std::endl;
        exit(-1);
    }

    std::cout << "网络接口: " << argv[1] << std::endl;

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    
    GO2SpeedControl test;
    test.Run();

    return 0;
}
