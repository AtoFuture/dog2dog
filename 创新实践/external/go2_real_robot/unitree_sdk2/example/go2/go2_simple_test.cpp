/**********************************************************************
 GO2 Simple Test - 室内安全测试程序
 用于测试基础指令：站立、趴下、状态查询等
***********************************************************************/

#include <iostream>
#include <string>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

class GO2SimpleTest
{
public:
    GO2SimpleTest()
    {
        sport_client.SetTimeout(10.0f);
        sport_client.Init();

        state_client.SetTimeout(10.0f);
        state_client.Init();

        suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&GO2SimpleTest::StateHandler, this, std::placeholders::_1), 1);
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

    // 阻尼模式（关节放松）
    void Damp()
    {
        std::cout << "\n>> 执行：阻尼模式（关节放松）" << std::endl;
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

    // 显示菜单
    void ShowMenu()
    {
        std::cout << "\n========== GO2 室内测试菜单 ==========" << std::endl;
        std::cout << "1 - 查询机器人状态" << std::endl;
        std::cout << "2 - 查询服务状态" << std::endl;
        std::cout << "3 - 站立 (StandUp)" << std::endl;
        std::cout << "4 - 趴下 (StandDown)" << std::endl;
        std::cout << "5 - 平衡站立 (BalanceStand)" << std::endl;
        std::cout << "6 - 阻尼模式 (Damp) - 关节放松" << std::endl;
        std::cout << "7 - 恢复站立 (RecoveryStand)" << std::endl;
        std::cout << "8 - 停止运动 (StopMove)" << std::endl;
        std::cout << "q - 退出程序" << std::endl;
        std::cout << "=======================================" << std::endl;
        std::cout << "请输入指令编号: ";
    }

    void Run()
    {
        sleep(1); // 等待1秒获取稳定状态
        
        std::cout << "\nGO2 室内测试程序已启动" << std::endl;
        std::cout << "安全提示：请在宽敞区域测试，确保周围无人" << std::endl;

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
                Damp();
            }
            else if (input == "7")
            {
                RecoveryStand();
            }
            else if (input == "8")
            {
                StopMove();
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
    
    GO2SimpleTest test;
    test.Run();

    return 0;
}
