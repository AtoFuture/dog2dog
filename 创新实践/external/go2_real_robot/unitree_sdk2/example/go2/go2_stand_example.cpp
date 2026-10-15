#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

// 🧑‍🍼发送“控制指令”的通道
#define TOPIC_LOWCMD "rt/lowcmd"
// 🤖发送“状态反馈”的通道
#define TOPIC_LOWSTATE "rt/lowstate"

// 魔数，表示“不控制位置”
constexpr double PosStopF = (2.146E+9f);
// 魔数，表示“不控制速度”
constexpr double VelStopF = (16000.0f);

// 总控制类
class Custom
{
public:
    explicit Custom(){}
    ~Custom(){}

    // 初始化通信、发布订阅，并释放当前机器人控制权
    void Init();
    // 启动周期控制线程，开始周期性发送控制指令（2ms循环）
    void Start();
    
private:
    // 初始化电机控制指令结构，设置安全默认状态（不运动）
    void InitLowCmd();
    // 接收机器人状态消息回调函数，用于更新本地low_state缓存
    void LowStateMessageHandler(const void* messages);
    // 核心控制循环：读取状态 → 计算轨迹 → 发布电机控制指令（2ms执行一次）
    void LowCmdWrite();
    // 查询当前机器人是否被其他运控服务占用（判断控制权状态）
    int queryMotionStatus();
    // 根据机器人模式返回对应运控服务名称（用于控制权识别）
    std::string queryServiceName(std::string form,std::string name);
 
private:
    float Kp = 60.0; // 位置增益：决定关节回到目标位置的“力度/刚性”
    float Kd = 5.0; // 阻尼增益：抑制运动过程中的抖动与过冲，使动作更平稳
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;

    // 控制循环计数器
    int motiontime = 0;

    // 控制循环时间间隔（2ms）
    float dt = 0.002; // 0.001~0.01

    // 控制“运控服务切换”
    MotionSwitcherClient msc;

    // 发给机器狗的“动作指令”
    unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    // 🐶实时反馈数据
    unitree_go::msg::dds_::LowState_ low_state{};  // default init

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;

    // 机器人三组目标姿态（核心动作）
    // 四足机器狗（每条3个关节）
    float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

    float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                              0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

    float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                              -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

    // 初始关节位置
    float _startPos[12];
    float _duration_1 = 500;   
    float _duration_2 = 500; 
    float _duration_3 = 1000;   
    float _duration_4 = 900;   
    float _percent_1 = 0;    
    float _percent_2 = 0;    
    float _percent_3 = 0;    
    float _percent_4 = 0;    

    // 是否第一次进入控制循环
    bool firstRun = true;

    // 动作是否完成
    bool done = false;
};

// 对机器人控制指令做 CRC 校验，保证数据在网络传输中没有损坏
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

// 初始化DDS发布/订阅通道，并循环释放机器人当前运控服务直到成功
void Custom::Init()
{
    InitLowCmd();

    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*init MotionSwitcherClient*/
    msc.SetTimeout(10.0f); 
    msc.Init();
    /*Shut down motion control-related service*/
    while(queryMotionStatus())
    {
        std::cout << "Try to deactivate the motion control-related service." << std::endl;
        // 释放控制权
        int32_t ret = msc.ReleaseMode(); 
        if (ret == 0) {
            std::cout << "ReleaseMode succeeded." << std::endl;
        } else {
            std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
        }
        sleep(5);
    }
}

// 初始化low_cmd指令为安全空状态
void Custom::InitLowCmd()
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

// 检查当前机器人是否还有“主运控服务”在运行，并返回是否已被占用（1=占用，0=已释放可控）
int Custom::queryMotionStatus()
{
    std::string robotForm,motionName;
    int motionStatus;
    int32_t ret = msc.CheckMode(robotForm,motionName);
    if (ret == 0) {
        std::cout << "CheckMode succeeded." << std::endl;
    } else {
        std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }
    if(motionName.empty())
    {
        std::cout << "The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = queryServiceName(robotForm,motionName);
        std::cout << "Service: "<< serviceName<< " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

// 将底层运动模式代码映射为可读的服务名称
std::string Custom::queryServiceName(std::string form,std::string name)
{
    if(form == "0")
    {
        if(name == "normal" ) return "sport_mode"; 
        if(name == "ai" ) return "ai_sport"; 
        if(name == "advanced" ) return "advanced_sport"; 
    }
    else
    {
        if(name == "ai-w" ) return "wheeled_sport(go2W)"; 
        if(name == "normal-w" ) return "wheeled_sport(b2W)";
    }
    return "";
}

// 启动一个周期性线程，定时执行 LowCmdWrite()，持续向机器人发送控制指令。
void Custom::Start()
{
    /*loop publishing thread*/
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &Custom::LowCmdWrite, this);
}

// 实时更新机器人当前状态数据（关节角、IMU、足端力等），供控制逻辑读取使用。
void Custom::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

// 让机器人按照预设轨迹平滑运动，同时实时发送控制指令，使Go2完成连续动作
void Custom::LowCmdWrite()
{
    if(_percent_4<1)
    {
        std::cout<<"Read sensor data example: "<<std::endl;
        std::cout<<"Joint 0 pos: "<<low_state.motor_state()[0].q()<<std::endl;
        std::cout<<"Imu accelerometer : "<<"x: "<<low_state.imu_state().accelerometer()[0]<<" y: "<<low_state.imu_state().accelerometer()[1]<<" z: "<<low_state.imu_state().accelerometer()[2]<<std::endl;
        std::cout<<"Foot force "<<low_state.foot_force()[0]<<std::endl;
        std::cout<<std::endl;
    }
    if((_percent_4 == 1) && ( done == false))
    {
        std::cout<<"The example is done! "<<std::endl;
        std::cout<<std::endl;
        done = true;
    }

    motiontime++;
    if(motiontime>=500)
    {
        if(firstRun)
        {
            for(int i = 0; i < 12; i++)
            {
                _startPos[i] = low_state.motor_state()[i].q();
            }
            firstRun = false;
        }

        _percent_1 += (float)1 / _duration_1;
        _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
        if (_percent_1 < 1)
        {
            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_1) * _startPos[j] + _percent_1 * _targetPos_1[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        
        }
        if ((_percent_1 == 1)&&(_percent_2 < 1))
        {
            _percent_2 += (float)1 / _duration_2;
            _percent_2 = _percent_2 > 1 ? 1 : _percent_2;

            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_2) * _targetPos_1[j] + _percent_2 * _targetPos_2[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }

        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3<1))
        {
            _percent_3 += (float)1 / _duration_3;
            _percent_3 = _percent_3 > 1 ? 1 : _percent_3;

            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() =  _targetPos_2[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }
        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3==1)&&((_percent_4<=1)))
        {
            _percent_4 += (float)1 / _duration_4;
            _percent_4 = _percent_4 > 1 ? 1 : _percent_4;
            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_4) * _targetPos_2[j] + _percent_4 * _targetPos_3[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }
        low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    
        lowcmd_publisher->Write(low_cmd);
    }
   
}

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ChannelFactory::Instance()->Init(0, argv[1]);

    Custom custom;
    custom.Init();
    custom.Start();
    
    while (1)
    {
        sleep(10);
    }

    return 0;
}
