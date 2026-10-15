/**********************************************************************
 GO2 抬腿测试程序 v2 - 底层电机控制 + 平衡站立
 功能：先平衡站立，再控制单腿抬起，验证稳定性和高度
 注意：需要与机械狗在同一局域网
***********************************************************************/

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <vector>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// 电机索引定义
// Leg 0 (FR): 0=Hip, 1=Thigh, 2=Calf
// Leg 1 (FL): 3=Hip, 4=Thigh, 5=Calf
// Leg 2 (RR): 6=Hip, 7=Thigh, 8=Calf
// Leg 3 (RL): 9=Hip, 10=Thigh, 11=Calf

// 选择要抬的腿：0=FR, 1=FL, 2=RR, 3=RL
#define TEST_LEG 1  // 默认抬左前腿

class LegLiftTest
{
public:
    LegLiftTest() {}
    ~LegLiftTest() {}

    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void HighStateMessageHandler(const void* messages);
    void LowCmdWrite();

private:
    float qInit[12] = {0};      // 12个关节初始角度
    float qDes[12] = {0};       // 12个关节目标角度
    float Kp[12] = {0};         // 位置增益
    float Kd[12] = {0};         // 速度增益
    int motiontime = 0;
    int lift_phase = 0;         // 抬腿阶段: 0=初始化, 1=抬起, 2=保持, 3=放下
    int phase_counter = 0;
    float dt = 0.002;           // 控制周期 2ms (500Hz)

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};
    unitree_go::msg::dds_::SportModeState_ high_state{};

    SportClient sport_client;

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> highstate_subscriber;
    ThreadPtr lowCmdWriteThreadPtr;
};

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

void LegLiftTest::Init()
{
    InitLowCmd();

    // 初始化 SportClient
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&LegLiftTest::LowStateMessageHandler, this, std::placeholders::_1), 1);

    highstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    highstate_subscriber->InitChannel(std::bind(&LegLiftTest::HighStateMessageHandler, this, std::placeholders::_1), 1);

    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("legliftcmd", UT_CPU_ID_NONE, 2000, &LegLiftTest::LowCmdWrite, this);
}

void LegLiftTest::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void LegLiftTest::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void LegLiftTest::HighStateMessageHandler(const void* message)
{
    high_state = *(unitree_go::msg::dds_::SportModeState_*)message;
}

void LegLiftTest::LowCmdWrite()
{
    motiontime++;

    const char* leg_names[] = {"右前腿(FR)", "左前腿(FL)", "右后腿(RR)", "左后腿(RL)"};
    int test_leg = TEST_LEG;

    // 站立状态的关节角度（宇树官方推荐值）
    // FR: Hip=0.0, Thigh=1.0, Calf=-2.0
    // FL: Hip=0.0, Thigh=1.0, Calf=-2.0
    // RR: Hip=0.0, Thigh=1.0, Calf=-2.0
    // RL: Hip=0.0, Thigh=1.0, Calf=-2.0
    float stand_q[12] = {
        0.0, 1.0, -2.0,   // FR
        0.0, 1.0, -2.0,   // FL
        0.0, 1.0, -2.0,   // RR
        0.0, 1.0, -2.0    // RL
    };

    // 阶段0: 使用 SportClient 平衡站立 (0-500帧 = 1秒)
    if (motiontime >= 0 && motiontime < 500)
    {
        if (motiontime == 1)
        {
            std::cout << "\n========== GO2 抬腿测试 v2 ==========" << std::endl;
            std::cout << "测试腿: " << leg_names[test_leg] << std::endl;
            std::cout << "阶段: 平衡站立..." << std::endl;
            
            // 调用 SportClient 平衡站立
            sport_client.BalanceStand();
        }
        
        // 此阶段不发送底层指令，让 SportClient 控制
        return;
    }

    // 阶段1: 记录站立位置并过渡到底层控制 (500-700帧 = 0.4秒)
    if (motiontime >= 500 && motiontime < 700)
    {
        if (motiontime == 500)
        {
            std::cout << "\n阶段: 记录站立位置，过渡到底层控制..." << std::endl;
            
            // 记录当前所有关节角度
            for (int i = 0; i < 12; i++)
            {
                qInit[i] = low_state.motor_state()[i].q();
                qDes[i] = qInit[i];
            }
            
            std::cout << "  测试腿初始角度:" << std::endl;
            std::cout << "    Hip: " << qInit[test_leg*3] << " rad" << std::endl;
            std::cout << "    Thigh: " << qInit[test_leg*3+1] << " rad" << std::endl;
            std::cout << "    Calf: " << qInit[test_leg*3+2] << " rad" << std::endl;
        }

        int count = motiontime - 500;
        double rate = count / 200.0;
        rate = std::min(rate, 1.0);

        // 逐渐过渡到站立位置
        for (int i = 0; i < 12; i++)
        {
            qDes[i] = jointLinearInterpolation(qInit[i], stand_q[i], rate);
        }

        // 所有关节使用中等增益保持站立
        for (int i = 0; i < 12; i++)
        {
            Kp[i] = 30.0;
            Kd[i] = 2.0;
        }
    }
    // 阶段2: 保持站立稳定 (700-1200帧 = 1秒)
    else if (motiontime >= 700 && motiontime < 1200)
    {
        if (motiontime == 700)
        {
            std::cout << "\n阶段: 保持站立稳定..." << std::endl;
        }

        // 保持站立位置
        for (int i = 0; i < 12; i++)
        {
            qDes[i] = stand_q[i];
        }

        for (int i = 0; i < 12; i++)
        {
            Kp[i] = 40.0;
            Kd[i] = 3.0;
        }
    }
    // 阶段3: 抬起测试腿 (1200-1700帧 = 1秒)
    else if (motiontime >= 1200 && motiontime < 1700)
    {
        if (motiontime == 1200)
        {
            lift_phase = 1;
            phase_counter = 0;
            std::cout << "\n阶段: 开始抬腿..." << std::endl;
        }

        phase_counter++;
        double rate = phase_counter / 500.0;

        // 测试腿抬腿
        int hip_idx = test_leg * 3;
        int thigh_idx = test_leg * 3 + 1;
        int calf_idx = test_leg * 3 + 2;

        float hip_target = stand_q[hip_idx] + 0.2;      // Hip 偏 0.2 rad
        float thigh_target = stand_q[thigh_idx] - 0.6;   // Thigh 抬 0.6 rad
        float calf_target = stand_q[calf_idx] + 0.4;     // Calf 弯曲 0.4 rad

        qDes[hip_idx] = jointLinearInterpolation(stand_q[hip_idx], hip_target, rate);
        qDes[thigh_idx] = jointLinearInterpolation(stand_q[thigh_idx], thigh_target, rate);
        qDes[calf_idx] = jointLinearInterpolation(stand_q[calf_idx], calf_target, rate);

        // 测试腿高增益
        Kp[hip_idx] = 40.0;   Kp[thigh_idx] = 80.0;   Kp[calf_idx] = 100.0;
        Kd[hip_idx] = 3.0;    Kd[thigh_idx] = 4.0;    Kd[calf_idx] = 6.0;

        // 其他腿保持站立（高增益维持稳定）
        for (int leg = 0; leg < 4; leg++)
        {
            if (leg == test_leg) continue;
            
            int h = leg * 3;
            int t = leg * 3 + 1;
            int c = leg * 3 + 2;
            
            qDes[h] = stand_q[h];
            qDes[t] = stand_q[t];
            qDes[c] = stand_q[c];
            
            Kp[h] = 50.0;   Kp[t] = 100.0;   Kp[c] = 80.0;
            Kd[h] = 4.0;    Kd[t] = 5.0;     Kd[c] = 5.0;
        }

        if (phase_counter % 100 == 0)
        {
            std::cout << "  抬腿进度: " << (int)(rate * 100) << "%" << std::endl;
        }
    }
    // 阶段4: 保持抬腿状态 (1700-2700帧 = 2秒)
    else if (motiontime >= 1700 && motiontime < 2700)
    {
        if (motiontime == 1700)
        {
            lift_phase = 2;
            phase_counter = 0;
            std::cout << "\n阶段: 保持抬腿状态，记录稳定性数据..." << std::endl;
        }

        phase_counter++;

        int hip_idx = test_leg * 3;
        int thigh_idx = test_leg * 3 + 1;
        int calf_idx = test_leg * 3 + 2;

        float hip_target = stand_q[hip_idx] + 0.2;
        float thigh_target = stand_q[thigh_idx] - 0.6;
        float calf_target = stand_q[calf_idx] + 0.4;

        qDes[hip_idx] = hip_target;
        qDes[thigh_idx] = thigh_target;
        qDes[calf_idx] = calf_target;

        Kp[hip_idx] = 50.0;   Kp[thigh_idx] = 100.0;   Kp[calf_idx] = 120.0;
        Kd[hip_idx] = 4.0;    Kd[thigh_idx] = 5.0;     Kd[calf_idx] = 7.0;

        // 其他腿保持站立
        for (int leg = 0; leg < 4; leg++)
        {
            if (leg == test_leg) continue;
            
            int h = leg * 3;
            int t = leg * 3 + 1;
            int c = leg * 3 + 2;
            
            qDes[h] = stand_q[h];
            qDes[t] = stand_q[t];
            qDes[c] = stand_q[c];
            
            Kp[h] = 60.0;   Kp[t] = 120.0;   Kp[c] = 100.0;
            Kd[h] = 5.0;    Kd[t] = 6.0;     Kd[c] = 6.0;
        }

        // 每500帧打印一次状态
        if (phase_counter % 500 == 0)
        {
            float current_hip = low_state.motor_state()[hip_idx].q();
            float current_thigh = low_state.motor_state()[thigh_idx].q();
            float current_calf = low_state.motor_state()[calf_idx].q();

            std::cout << "\n  [稳定性检查] 第 " << phase_counter/500 << " 秒:" << std::endl;
            std::cout << "    Hip 角度: " << current_hip << " rad (目标: " << hip_target << ")" << std::endl;
            std::cout << "    Thigh 角度: " << current_thigh << " rad (目标: " << thigh_target << ")" << std::endl;
            std::cout << "    Calf 角度: " << current_calf << " rad (目标: " << calf_target << ")" << std::endl;
            std::cout << "    Hip 误差: " << fabs(current_hip - hip_target) << " rad" << std::endl;
            std::cout << "    Thigh 误差: " << fabs(current_thigh - thigh_target) << " rad" << std::endl;
            std::cout << "    Calf 误差: " << fabs(current_calf - calf_target) << " rad" << std::endl;
            
            // 估算抬腿高度
            float thigh_length = 0.213;
            float lift_height = thigh_length * (1 - cos(current_thigh - stand_q[thigh_idx]));
            std::cout << "    估算抬腿高度: " << (lift_height * 100) << " cm" << std::endl;
        }
    }
    // 阶段5: 放下腿 (2700-3200帧 = 1秒)
    else if (motiontime >= 2700 && motiontime < 3200)
    {
        if (motiontime == 2700)
        {
            lift_phase = 3;
            phase_counter = 0;
            std::cout << "\n阶段: 开始放下腿..." << std::endl;
        }

        phase_counter++;
        double rate = phase_counter / 500.0;

        int hip_idx = test_leg * 3;
        int thigh_idx = test_leg * 3 + 1;
        int calf_idx = test_leg * 3 + 2;

        float hip_target = stand_q[hip_idx] + 0.2;
        float thigh_target = stand_q[thigh_idx] - 0.6;
        float calf_target = stand_q[calf_idx] + 0.4;

        qDes[hip_idx] = jointLinearInterpolation(hip_target, stand_q[hip_idx], rate);
        qDes[thigh_idx] = jointLinearInterpolation(thigh_target, stand_q[thigh_idx], rate);
        qDes[calf_idx] = jointLinearInterpolation(calf_target, stand_q[calf_idx], rate);

        Kp[hip_idx] = 40.0;   Kp[thigh_idx] = 80.0;   Kp[calf_idx] = 100.0;
        Kd[hip_idx] = 3.0;    Kd[thigh_idx] = 4.0;    Kd[calf_idx] = 6.0;

        // 其他腿保持站立
        for (int leg = 0; leg < 4; leg++)
        {
            if (leg == test_leg) continue;
            
            int h = leg * 3;
            int t = leg * 3 + 1;
            int c = leg * 3 + 2;
            
            qDes[h] = stand_q[h];
            qDes[t] = stand_q[t];
            qDes[c] = stand_q[c];
            
            Kp[h] = 50.0;   Kp[t] = 100.0;   Kp[c] = 80.0;
            Kd[h] = 4.0;    Kd[t] = 5.0;     Kd[c] = 5.0;
        }

        if (phase_counter % 100 == 0)
        {
            std::cout << "  放下进度: " << (int)(rate * 100) << "%" << std::endl;
        }
    }
    // 阶段6: 回到站立位置并保持 (3200帧以后)
    else if (motiontime >= 3200)
    {
        if (motiontime == 3200)
        {
            std::cout << "\n========== 抬腿测试完成 ==========" << std::endl;
            std::cout << "保持站立 3 秒后退出..." << std::endl;
        }

        // 保持站立位置
        for (int i = 0; i < 12; i++)
        {
            qDes[i] = stand_q[i];
        }

        for (int i = 0; i < 12; i++)
        {
            Kp[i] = 40.0;
            Kd[i] = 3.0;
        }

        // 运行3秒后退出
        if (motiontime > 4700)
        {
            std::cout << "测试结束，退出程序。" << std::endl;
            exit(0);
        }
    }

    // 写入所有12个电机的控制指令
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].q() = qDes[i];
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = Kp[i];
        low_cmd.motor_cmd()[i].kd() = Kd[i];
        low_cmd.motor_cmd()[i].tau() = 0;
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    
    lowcmd_publisher->Write(low_cmd);
}

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        std::cout << "  leg: 修改 TEST_LEG 宏选择抬腿 (0=FR, 1=FL, 2=RR, 3=RL)" << std::endl;
        exit(-1);
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    LegLiftTest test;
    test.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
