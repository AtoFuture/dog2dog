/**********************************************************************
 GO2 握手程序 v2 - 参考官方文档重写
 功能：机械狗抬起前腿与人握手
 原理：SportClient维持平衡 + 底层控制单腿摆动
 注意：需要与机械狗在同一局域网
***********************************************************************/

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
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

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// 电机索引定义
// Leg 0 (FR): 0=Hip, 1=Thigh, 2=Calf
// Leg 1 (FL): 3=Hip, 4=Thigh, 5=Calf
// Leg 2 (RR): 6=Hip, 7=Thigh, 8=Calf
// Leg 3 (RL): 9=Hip, 10=Thigh, 11=Calf

class HandshakeTest
{
public:
    HandshakeTest() {}
    ~HandshakeTest() {}

    void Init(const char* network_interface);

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();
    double jointLinearInterpolation(double initPos, double targetPos, double rate);

private:
    float qInit[3] = {0};       // 测试腿3个关节初始角度
    float qDes[3] = {0};        // 测试腿3个关节目标角度
    float sin_mid_q[3] = {0.0, 1.2, -2.0}; // 中间角度
    float Kp[3] = {0};          // 测试腿位置增益
    float Kd[3] = {0};          // 测试腿速度增益
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002;           // 控制周期 2ms (500Hz)

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    SportClient sport_client;

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
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

double HandshakeTest::jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

void HandshakeTest::Init(const char* network_interface)
{
    InitLowCmd();

    // 初始化 SportClient 用于维持平衡
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&HandshakeTest::LowStateMessageHandler, this, std::placeholders::_1), 1);

    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("handshakecmd", UT_CPU_ID_NONE, 2000, &HandshakeTest::LowCmdWrite, this);
}

void HandshakeTest::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void HandshakeTest::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void HandshakeTest::LowCmdWrite()
{
    motiontime++;

    // 选择左前腿 FL (关节 3,4,5)
    int test_leg = 1;
    int hip_idx = test_leg * 3;
    int thigh_idx = test_leg * 3 + 1;
    int calf_idx = test_leg * 3 + 2;

    // 阶段0: 使用 SportClient 平衡站立 (0-500帧 = 1秒)
    if (motiontime >= 0 && motiontime < 500)
    {
        if (motiontime == 1)
        {
            std::cout << "\n========== GO2 握手程序 v2 ==========" << std::endl;
            std::cout << "阶段0: 平衡站立 (SportClient)..." << std::endl;
            sport_client.BalanceStand();
        }
        // 此阶段只发送初始化指令，不控制关节
        low_cmd.crc() = crc32_core((uint32_t*)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
        lowcmd_publisher->Write(low_cmd);
        return;
    }

    // 阶段1: 记录初始位置并移动到起始点 (500-900帧 = 0.8秒)
    if (motiontime >= 500 && motiontime < 900)
    {
        if (motiontime == 500)
        {
            std::cout << "\n阶段1: 记录初始位置，移动到起始点..." << std::endl;
            // 记录测试腿初始角度
            qInit[0] = low_state.motor_state()[hip_idx].q();
            qInit[1] = low_state.motor_state()[thigh_idx].q();
            qInit[2] = low_state.motor_state()[calf_idx].q();

            std::cout << "  初始角度 - Hip: " << qInit[0] << ", Thigh: " << qInit[1] << ", Calf: " << qInit[2] << std::endl;
        }

        rate_count++;
        double rate = rate_count / 400.0;
        rate = std::min(rate, 1.0);

        // 低增益移动到中间位置
        Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
        Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;

        qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
        qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
        qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);

        // 只控制测试腿的3个关节
        low_cmd.motor_cmd()[hip_idx].q() = qDes[0];
        low_cmd.motor_cmd()[hip_idx].dq() = 0;
        low_cmd.motor_cmd()[hip_idx].kp() = Kp[0];
        low_cmd.motor_cmd()[hip_idx].kd() = Kd[0];
        low_cmd.motor_cmd()[hip_idx].tau() = 0;

        low_cmd.motor_cmd()[thigh_idx].q() = qDes[1];
        low_cmd.motor_cmd()[thigh_idx].dq() = 0;
        low_cmd.motor_cmd()[thigh_idx].kp() = Kp[1];
        low_cmd.motor_cmd()[thigh_idx].kd() = Kd[1];
        low_cmd.motor_cmd()[thigh_idx].tau() = 0;

        low_cmd.motor_cmd()[calf_idx].q() = qDes[2];
        low_cmd.motor_cmd()[calf_idx].dq() = 0;
        low_cmd.motor_cmd()[calf_idx].kp() = Kp[2];
        low_cmd.motor_cmd()[calf_idx].kd() = Kd[2];
        low_cmd.motor_cmd()[calf_idx].tau() = 0;

        // 其他关节保持 PosStopF 状态
    }

    // 阶段2: 抬起腿 (900-1400帧 = 1秒)
    else if (motiontime >= 900 && motiontime < 1400)
    {
        if (motiontime == 900)
        {
            std::cout << "\n阶段2: 抬起左前腿..." << std::endl;
            rate_count = 0;
        }

        rate_count++;
        double rate = rate_count / 500.0;
        rate = std::min(rate, 1.0);

        // 抬腿目标角度
        float hip_target = sin_mid_q[0] + 0.3;
        float thigh_target = sin_mid_q[1] - 0.6;
        float calf_target = sin_mid_q[2] + 0.4;

        // 增大增益
        Kp[0] = 10.0; Kp[1] = 10.0; Kp[2] = 10.0;
        Kd[0] = 2.0;  Kd[1] = 2.0;  Kd[2] = 2.0;

        qDes[0] = jointLinearInterpolation(sin_mid_q[0], hip_target, rate);
        qDes[1] = jointLinearInterpolation(sin_mid_q[1], thigh_target, rate);
        qDes[2] = jointLinearInterpolation(sin_mid_q[2], calf_target, rate);

        low_cmd.motor_cmd()[hip_idx].q() = qDes[0];
        low_cmd.motor_cmd()[hip_idx].dq() = 0;
        low_cmd.motor_cmd()[hip_idx].kp() = Kp[0];
        low_cmd.motor_cmd()[hip_idx].kd() = Kd[0];
        low_cmd.motor_cmd()[hip_idx].tau() = 0;

        low_cmd.motor_cmd()[thigh_idx].q() = qDes[1];
        low_cmd.motor_cmd()[thigh_idx].dq() = 0;
        low_cmd.motor_cmd()[thigh_idx].kp() = Kp[1];
        low_cmd.motor_cmd()[thigh_idx].kd() = Kd[1];
        low_cmd.motor_cmd()[thigh_idx].tau() = 0;

        low_cmd.motor_cmd()[calf_idx].q() = qDes[2];
        low_cmd.motor_cmd()[calf_idx].dq() = 0;
        low_cmd.motor_cmd()[calf_idx].kp() = Kp[2];
        low_cmd.motor_cmd()[calf_idx].kd() = Kd[2];
        low_cmd.motor_cmd()[calf_idx].tau() = 0;

        if (rate_count % 250 == 0)
        {
            std::cout << "  抬腿进度: " << (int)(rate * 100) << "%" << std::endl;
        }
    }

    // 阶段3: 握手摆动 (1400-2900帧 = 3秒，约3次握手)
    else if (motiontime >= 1400 && motiontime < 2900)
    {
        if (motiontime == 1400)
        {
            std::cout << "\n阶段3: 开始握手摆动..." << std::endl;
            sin_count = 0;
        }

        sin_count++;
        double t = dt * sin_count;

        // 正弦波握手摆动 (频率 1Hz)
        float freq_Hz = 1;
        float freq_rad = freq_Hz * 2 * M_PI;
        float sin_joint1 = 0.3 * sin(t * freq_rad);  // Hip 摆动
        float sin_joint2 = -0.5 * sin(t * freq_rad); // Thigh 摆动
        float sin_joint3 = 0.3 * sin(t * freq_rad);  // Calf 摆动

        float hip_target = sin_mid_q[0] + 0.3 + sin_joint1;
        float thigh_target = sin_mid_q[1] - 0.6 + sin_joint2;
        float calf_target = sin_mid_q[2] + 0.4 + sin_joint3;

        Kp[0] = 10.0; Kp[1] = 10.0; Kp[2] = 10.0;
        Kd[0] = 2.0;  Kd[1] = 2.0;  Kd[2] = 2.0;

        low_cmd.motor_cmd()[hip_idx].q() = hip_target;
        low_cmd.motor_cmd()[hip_idx].dq() = 0;
        low_cmd.motor_cmd()[hip_idx].kp() = Kp[0];
        low_cmd.motor_cmd()[hip_idx].kd() = Kd[0];
        low_cmd.motor_cmd()[hip_idx].tau() = 0;

        low_cmd.motor_cmd()[thigh_idx].q() = thigh_target;
        low_cmd.motor_cmd()[thigh_idx].dq() = 0;
        low_cmd.motor_cmd()[thigh_idx].kp() = Kp[1];
        low_cmd.motor_cmd()[thigh_idx].kd() = Kd[1];
        low_cmd.motor_cmd()[thigh_idx].tau() = 0;

        low_cmd.motor_cmd()[calf_idx].q() = calf_target;
        low_cmd.motor_cmd()[calf_idx].dq() = 0;
        low_cmd.motor_cmd()[calf_idx].kp() = Kp[2];
        low_cmd.motor_cmd()[calf_idx].kd() = Kd[2];
        low_cmd.motor_cmd()[calf_idx].tau() = 0;

        if (sin_count % 500 == 0)
        {
            int shake_num = sin_count / 500 + 1;
            std::cout << "  握手次数: " << shake_num << std::endl;
        }
    }

    // 阶段4: 放下腿 (2900-3400帧 = 0.5秒)
    else if (motiontime >= 2900 && motiontime < 3400)
    {
        if (motiontime == 2900)
        {
            std::cout << "\n阶段4: 放下腿..." << std::endl;
            rate_count = 0;
        }

        rate_count++;
        double rate = rate_count / 500.0;
        rate = std::min(rate, 1.0);

        float hip_current = sin_mid_q[0] + 0.3;
        float thigh_current = sin_mid_q[1] - 0.6;
        float calf_current = sin_mid_q[2] + 0.4;

        Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
        Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;

        qDes[0] = jointLinearInterpolation(hip_current, sin_mid_q[0], rate);
        qDes[1] = jointLinearInterpolation(thigh_current, sin_mid_q[1], rate);
        qDes[2] = jointLinearInterpolation(calf_current, sin_mid_q[2], rate);

        low_cmd.motor_cmd()[hip_idx].q() = qDes[0];
        low_cmd.motor_cmd()[hip_idx].dq() = 0;
        low_cmd.motor_cmd()[hip_idx].kp() = Kp[0];
        low_cmd.motor_cmd()[hip_idx].kd() = Kd[0];
        low_cmd.motor_cmd()[hip_idx].tau() = 0;

        low_cmd.motor_cmd()[thigh_idx].q() = qDes[1];
        low_cmd.motor_cmd()[thigh_idx].dq() = 0;
        low_cmd.motor_cmd()[thigh_idx].kp() = Kp[1];
        low_cmd.motor_cmd()[thigh_idx].kd() = Kd[1];
        low_cmd.motor_cmd()[thigh_idx].tau() = 0;

        low_cmd.motor_cmd()[calf_idx].q() = qDes[2];
        low_cmd.motor_cmd()[calf_idx].dq() = 0;
        low_cmd.motor_cmd()[calf_idx].kp() = Kp[2];
        low_cmd.motor_cmd()[calf_idx].kd() = Kd[2];
        low_cmd.motor_cmd()[calf_idx].tau() = 0;

        if (rate_count % 250 == 0)
        {
            std::cout << "  放下进度: " << (int)(rate * 100) << "%" << std::endl;
        }
    }

    // 阶段5: 回到中间位置后退出 (3400-3900帧)
    else if (motiontime >= 3400 && motiontime < 3900)
    {
        if (motiontime == 3400)
        {
            std::cout << "\n阶段5: 握手完成，回到中间位置..." << std::endl;
            rate_count = 0;
        }

        rate_count++;
        double rate = rate_count / 500.0;
        rate = std::min(rate, 1.0);

        Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
        Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;

        qDes[0] = jointLinearInterpolation(sin_mid_q[0], qInit[0], rate);
        qDes[1] = jointLinearInterpolation(sin_mid_q[1], qInit[1], rate);
        qDes[2] = jointLinearInterpolation(sin_mid_q[2], qInit[2], rate);

        low_cmd.motor_cmd()[hip_idx].q() = qDes[0];
        low_cmd.motor_cmd()[hip_idx].dq() = 0;
        low_cmd.motor_cmd()[hip_idx].kp() = Kp[0];
        low_cmd.motor_cmd()[hip_idx].kd() = Kd[0];
        low_cmd.motor_cmd()[hip_idx].tau() = 0;

        low_cmd.motor_cmd()[thigh_idx].q() = qDes[1];
        low_cmd.motor_cmd()[thigh_idx].dq() = 0;
        low_cmd.motor_cmd()[thigh_idx].kp() = Kp[1];
        low_cmd.motor_cmd()[thigh_idx].kd() = Kd[1];
        low_cmd.motor_cmd()[thigh_idx].tau() = 0;

        low_cmd.motor_cmd()[calf_idx].q() = qDes[2];
        low_cmd.motor_cmd()[calf_idx].dq() = 0;
        low_cmd.motor_cmd()[calf_idx].kp() = Kp[2];
        low_cmd.motor_cmd()[calf_idx].kd() = Kd[2];
        low_cmd.motor_cmd()[calf_idx].tau() = 0;
    }

    // 退出
    else if (motiontime >= 3900)
    {
        std::cout << "\n========== 握手程序结束 ==========" << std::endl;
        exit(0);
    }

    low_cmd.crc() = crc32_core((uint32_t*)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    lowcmd_publisher->Write(low_cmd);
}

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0" << std::endl;
        exit(-1);
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    HandshakeTest test;
    test.Init(argv[1]);

    while (1)
    {
        sleep(10);
    }

    return 0;
}
