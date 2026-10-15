/**********************************************************************
 GO2 基础导航程序 - 阶段一
 功能：控制机械狗移动到指定坐标，并确认到达后的位置和姿态
 注意：需要与机械狗在同一局域网
 
 命令行参数说明：
   ./go2_navigation <networkInterface> [选项]
   
   必需参数：
     networkInterface    网卡名称（如 eth0）
   
   可选参数（未指定时使用默认值）：
     --target-x <value>      目标 x 坐标（米），默认 2.0
     --target-y <value>      目标 y 坐标（米），默认 0.0
     --target-yaw <value>    目标朝向（弧度），默认 0.0
     --pos-threshold <value> 位置判定阈值（米），默认 0.15
     --yaw-threshold <value> 朝向判定阈值（弧度），默认 0.1
     --max-speed <value>     最大移动速度（m/s），默认 0.5
     --rotation-speed <value> 最大旋转速度（rad/s），默认 1.0
     --timeout <value>       超时时间（秒），默认 30.0
     --gain-xy <value>       位置控制增益，默认 1.5
     --gain-yaw <value>      朝向控制增益，默认 1.2
   
   示例：
     # 使用默认参数前进 2 米
     ./go2_navigation eth0
     
     # 前进 3 米，左转 90 度
     ./go2_navigation eth0 --target-x 3.0 --target-yaw 1.57
     
     # 移动到 (1, 1) 位置，速度 0.3 m/s
     ./go2_navigation eth0 --target-x 1.0 --target-y 1.0 --max-speed 0.3
     
     # 严格判定到达（误差 < 5cm），超时 60 秒
     ./go2_navigation eth0 --target-x 5.0 --pos-threshold 0.05 --timeout 60
***********************************************************************/

#include <iostream>
#include <cmath>
#include <string>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/common/thread/thread.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;
using namespace unitree::robot;

// ============================================================================
// 参数配置结构体
// ============================================================================

struct NavigationConfig
{
    // 【目标位置参数】
    double target_x = 2.0;      // 目标 x 坐标（米），相对于程序启动时的位置
    double target_y = 0.0;      // 目标 y 坐标（米），相对于程序启动时的位置
    double target_yaw = 0.0;    // 目标朝向（弧度），相对于启动时的朝向

    // 【到达判定阈值】
    double position_threshold = 0.15;  // 位置判定阈值（米），<15cm 认为到达
    double yaw_threshold = 0.1;        // 朝向判定阈值（弧度），<5.7° 认为朝向正确

    // 【运动控制参数】
    double max_speed = 0.5;         // 最大移动速度（m/s）
    double rotation_speed = 1.0;    // 最大旋转速度（rad/s）

    // 【安全参数】
    double timeout_seconds = 30.0;  // 超时时间（秒）

    // 【控制器增益】
    double gain_xy = 1.5;   // 位置控制增益，越大响应越快
    double gain_yaw = 1.2;  // 朝向控制增益
};

// ============================================================================
// 命令行参数解析函数
// ============================================================================

void printUsage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " <networkInterface> [选项]" << std::endl;
    std::cout << std::endl;
    std::cout << "必需参数:" << std::endl;
    std::cout << "  networkInterface    网卡名称（如 eth0）" << std::endl;
    std::cout << std::endl;
    std::cout << "可选参数:" << std::endl;
    std::cout << "  --target-x <value>      目标 x 坐标（米），默认 2.0" << std::endl;
    std::cout << "  --target-y <value>      目标 y 坐标（米），默认 0.0" << std::endl;
    std::cout << "  --target-yaw <value>    目标朝向（弧度），默认 0.0" << std::endl;
    std::cout << "  --pos-threshold <value> 位置判定阈值（米），默认 0.15" << std::endl;
    std::cout << "  --yaw-threshold <value> 朝向判定阈值（弧度），默认 0.1" << std::endl;
    std::cout << "  --max-speed <value>     最大移动速度（m/s），默认 0.5" << std::endl;
    std::cout << "  --rotation-speed <value> 最大旋转速度（rad/s），默认 1.0" << std::endl;
    std::cout << "  --timeout <value>       超时时间（秒），默认 30.0" << std::endl;
    std::cout << "  --gain-xy <value>       位置控制增益，默认 1.5" << std::endl;
    std::cout << "  --gain-yaw <value>      朝向控制增益，默认 1.2" << std::endl;
    std::cout << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  # 使用默认参数前进 2 米" << std::endl;
    std::cout << "  " << program_name << " eth0" << std::endl;
    std::cout << std::endl;
    std::cout << "  # 前进 3 米，左转 90 度" << std::endl;
    std::cout << "  " << program_name << " eth0 --target-x 3.0 --target-yaw 1.57" << std::endl;
    std::cout << std::endl;
    std::cout << "  # 移动到 (1, 1) 位置，速度 0.3 m/s" << std::endl;
    std::cout << "  " << program_name << " eth0 --target-x 1.0 --target-y 1.0 --max-speed 0.3" << std::endl;
}

// 解析命令行参数，返回是否解析成功
bool parseArguments(int argc, const char** argv, NavigationConfig& config, std::string& network_interface)
{
    // 至少需要网卡名称
    if (argc < 2)
    {
        printUsage(argv[0]);
        return false;
    }

    // 第一个参数是网卡名称
    network_interface = argv[1];

    // 解析可选参数
    for (int i = 2; i < argc; i++)
    {
        std::string arg = argv[i];

        // 检查是否需要下一个参数
        if (i + 1 >= argc)
        {
            std::cout << "错误: 参数 " << arg << " 缺少值" << std::endl;
            printUsage(argv[0]);
            return false;
        }

        double value = std::stod(argv[i + 1]);  // 解析数值
        i++;  // 跳过值参数

        if (arg == "--target-x")
        {
            config.target_x = value;
        }
        else if (arg == "--target-y")
        {
            config.target_y = value;
        }
        else if (arg == "--target-yaw")
        {
            config.target_yaw = value;
        }
        else if (arg == "--pos-threshold")
        {
            config.position_threshold = value;
        }
        else if (arg == "--yaw-threshold")
        {
            config.yaw_threshold = value;
        }
        else if (arg == "--max-speed")
        {
            config.max_speed = value;
        }
        else if (arg == "--rotation-speed")
        {
            config.rotation_speed = value;
        }
        else if (arg == "--timeout")
        {
            config.timeout_seconds = value;
        }
        else if (arg == "--gain-xy")
        {
            config.gain_xy = value;
        }
        else if (arg == "--gain-yaw")
        {
            config.gain_yaw = value;
        }
        else
        {
            std::cout << "错误: 未知参数 " << arg << std::endl;
            printUsage(argv[0]);
            return false;
        }
    }

    return true;
}

// ============================================================================
// 导航控制类
// ============================================================================

class NavigationController
{
public:
    NavigationController() {}
    ~NavigationController() {}

    // 初始化函数：建立 DDS 连接、订阅状态、启动控制线程
    // config: 导航配置参数（从命令行解析）
    void Init(const char* network_interface, const NavigationConfig& config);

private:
    // 高层状态回调函数：接收机械狗的实时位置和姿态
    void HighStateHandler(const void* message);

    // 导航控制主循环：计算误差、发送速度指令、判定是否到达
    void NavigateToTarget();

    // 计算两点之间的欧几里得距离
    double calcDistance(double x1, double y1, double x2, double y2);

    // 计算两个角度之间的最小差值（处理角度环绕问题）
    double calcAngleDiff(double angle1, double angle2);

    // 打印当前位置和姿态信息（用于调试）
    void printState();

    // 线程指针（必须保存，否则线程会被立即销毁）
    unitree::common::ThreadPtr navThreadPtr;

private:
    // DDS 通信相关对象
    unitree_go::msg::dds_::SportModeState_ state;  // 存储机械狗的高层状态
    unitree::robot::go2::SportClient sport_client;  // 运动控制客户端
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;  // 状态订阅器

    // 初始状态记录（程序启动时的位置和朝向）
    double init_x = 0.0;
    double init_y = 0.0;
    double init_yaw = 0.0;

    // 控制参数
    float dt = 0.005;  // 控制周期（秒），范围 0.001~0.01，值越小控制越精细
    int motiontime = 0;  // 运动计时器（帧数，每帧 5ms）
    bool arrived = false;  // 到达标志位
    bool timed_out = false;  // 超时标志位

    // 导航配置（从命令行传入）
    NavigationConfig config;
};

// ============================================================================
// 初始化函数
// ============================================================================
void NavigationController::Init(const char* network_interface, const NavigationConfig& cfg)
{
    // 保存配置参数
    config = cfg;

    // 调试输出：打印接收到的配置
    std::cout << "\n[DEBUG] 接收到的配置:" << std::endl;
    std::cout << "  target_x=" << config.target_x << std::endl;
    std::cout << "  target_y=" << config.target_y << std::endl;
    std::cout << "  max_speed=" << config.max_speed << std::endl;
    std::cout << "  timeout=" << config.timeout_seconds << std::endl;

    // 初始化 SportClient 用于发送运动指令
    sport_client.SetTimeout(10.0f);  // 设置 RPC 超时为 10 秒
    sport_client.Init();

    // 订阅高层状态话题，获取机械狗的实时位置和姿态
    // 回调函数 HighStateHandler 会每 2ms 被调用一次
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&NavigationController::HighStateHandler, this, std::placeholders::_1), 1);

    // 等待状态数据就绪（最多等待 5 秒）
    std::cout << "等待机械狗状态数据..." << std::endl;
    int wait_count = 0;
    while (wait_count < 500)  // 最多等待 5 秒
    {
        // 检查 position 数据是否有效（非零或已更新）
        if (state.position().size() >= 3 && state.imu_state().rpy().size() >= 3)
        {
            // 确保数据不是全零（表示已收到真实数据）
            if (std::abs(state.position()[0]) > 0.001 || std::abs(state.position()[1]) > 0.001)
            {
                break;
            }
        }
        usleep(10000);  // 等待 10ms
        wait_count++;
    }

    if (wait_count >= 500)
    {
        std::cout << "错误: 无法获取机械狗状态数据，请检查连接" << std::endl;
        exit(-1);
    }

    std::cout << "状态数据已就绪" << std::endl;

    // 记录初始位置和朝向（作为坐标系原点）
    init_x = state.position()[0];
    init_y = state.position()[1];
    init_yaw = state.imu_state().rpy()[2];

    std::cout << "\n========== GO2 基础导航程序 ==========" << std::endl;
    std::cout << "初始位置: x=" << init_x << ", y=" << init_y << ", yaw=" << init_yaw << std::endl;
    std::cout << "目标位置: x=" << config.target_x << ", y=" << config.target_y << ", yaw=" << config.target_yaw << std::endl;
    std::cout << "最大速度: " << config.max_speed << " m/s | 超时: " << config.timeout_seconds << " s" << std::endl;
    std::cout << "======================================\n" << std::endl;

    // 启动平衡站立模式，准备接收运动指令
    std::cout << "[DEBUG] 调用 BalanceStand..." << std::endl;
    int32_t ret = sport_client.BalanceStand();
    std::cout << "[DEBUG] BalanceStand 返回: " << ret << std::endl;
    sleep(1);  // 等待机械狗站稳

    std::cout << "[DEBUG] 即将创建线程..." << std::endl;
    // 创建周期性线程，每 5ms 执行一次导航控制（必须保存返回值）
    navThreadPtr = unitree::common::CreateRecurrentThread(dt * 1000000, std::bind(&NavigationController::NavigateToTarget, this));
    std::cout << "[DEBUG] 线程已创建" << std::endl;
}

// ============================================================================
// 高层状态回调函数
// 功能：接收 DDS 传来的机械狗状态，更新到 state 变量中
// 触发频率：约 500Hz（每 2ms 一次）
// ============================================================================
void NavigationController::HighStateHandler(const void* message)
{
    state = *(unitree_go::msg::dds_::SportModeState_*)message;
}

// ============================================================================
// 距离计算函数
// 参数：(x1, y1) - 起点坐标，(x2, y2) - 终点坐标
// 返回：两点之间的欧几里得距离（米）
// ============================================================================
double NavigationController::calcDistance(double x1, double y1, double x2, double y2)
{
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// ============================================================================
// 角度差计算函数
// 参数：angle1 - 当前角度，angle2 - 目标角度（均为弧度）
// 返回：两个角度之间的最小差值（范围 [-π, π]）
// 说明：处理角度环绕问题，例如 179° 和 -179° 的差值应该是 2° 而不是 358°
// ============================================================================
double NavigationController::calcAngleDiff(double angle1, double angle2)
{
    double diff = angle2 - angle1;
    // 将差值归一化到 [-π, π] 范围
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

// ============================================================================
// 状态打印函数
// 功能：输出当前位置、姿态、速度等调试信息
// ============================================================================
void NavigationController::printState()
{
    // 安全检查
    if (state.position().size() < 3 || state.imu_state().rpy().size() < 3 || state.velocity().size() < 3)
    {
        std::cout << "\n--- 状态数据未就绪 ---" << std::endl;
        return;
    }

    // 计算相对于初始位置的坐标
    double current_x = state.position()[0] - init_x;
    double current_y = state.position()[1] - init_y;
    double current_yaw = state.imu_state().rpy()[2] - init_yaw;

    std::cout << "\n--- 当前状态 ---" << std::endl;
    std::cout << "位置: x=" << current_x << " m, y=" << current_y << " m" << std::endl;
    std::cout << "朝向: yaw=" << current_yaw << " rad (" << (current_yaw * 180.0 / M_PI) << "°)" << std::endl;
    std::cout << "速度: vx=" << state.velocity()[0] << " m/s, vy=" << state.velocity()[1] << " m/s" << std::endl;
    std::cout << "姿态: roll=" << state.imu_state().rpy()[0] << ", pitch=" << state.imu_state().rpy()[1] << std::endl;
}

// ============================================================================
// 导航控制主循环
// 功能：
//   1. 读取当前位置和姿态
//   2. 计算与目标点的误差
//   3. 根据误差计算速度指令（比例控制）
//   4. 发送速度指令给机械狗
//   5. 判定是否到达目标点
// 触发频率：每 5ms 一次（200Hz）
// ============================================================================
void NavigationController::NavigateToTarget()
{
    motiontime++;

    // 如果已经到达目标或超时，则停止控制
    if (arrived || timed_out)
    {
        if (motiontime % 1000 == 0)  // 每 5 秒打印一次状态
        {
            printState();
        }
        return;
    }

    // 安全检查：确保状态数据有效
    if (state.position().size() < 3 || state.imu_state().rpy().size() < 3 || state.velocity().size() < 3)
    {
        return;  // 数据未就绪，跳过本次循环
    }

    // 计算当前位置（在机器人本体坐标系中，相对于初始位置）
    // 根据官方文档：Move(vx,vy,vyaw) 使用 body coordinate system
    // state.position() 返回绝对坐标，需要转换到本体坐标系
    double dx = state.position()[0] - init_x;
    double dy = state.position()[1] - init_y;
    double current_yaw = state.imu_state().rpy()[2] - init_yaw;
    
    // 将世界坐标系中的偏移量 (dx, dy) 转换到机器人本体坐标系
    // 使用当前机器人的绝对朝向角
    double abs_yaw = state.imu_state().rpy()[2];
    double current_x = cos(abs_yaw) * dx + sin(abs_yaw) * dy;
    double current_y = -sin(abs_yaw) * dx + cos(abs_yaw) * dy;

    // 计算位置误差
    double error_x = config.target_x - current_x;
    double error_y = config.target_y - current_y;
    double error_yaw = calcAngleDiff(current_yaw, config.target_yaw);
    double distance_to_target = calcDistance(current_x, current_y, config.target_x, config.target_y);

    // 打印进度（每 500ms 一次）
    if (motiontime % 100 == 0)
    {
        std::cout << "距离目标: " << distance_to_target << " m | 朝向误差: " << (error_yaw * 180.0 / M_PI) << "°" << std::endl;
    }

    // 判定是否到达目标点
    if (distance_to_target < config.position_threshold && std::abs(error_yaw) < config.yaw_threshold)
    {
        arrived = true;
        sport_client.StopMove();  // 停止移动

        std::cout << "\n========== 到达目标点 ==========" << std::endl;
        printState();
        std::cout << "\n位置误差: " << distance_to_target << " m (< " << config.position_threshold << " m)" << std::endl;
        std::cout << "朝向误差: " << (error_yaw * 180.0 / M_PI) << "° (< " << (config.yaw_threshold * 180.0 / M_PI) << "°)" << std::endl;
        std::cout << "================================\n" << std::endl;

        // 保持站立 5 秒后退出程序
        sleep(5);
        exit(0);
        return;
    }

    // 检查是否超时
    double elapsed_time = motiontime * dt;  // 已运行时间（秒）
    if (elapsed_time > config.timeout_seconds)
    {
        timed_out = true;
        sport_client.StopMove();

        std::cout << "\n========== 超时 ==========" << std::endl;
        std::cout << "运行时间: " << elapsed_time << " s (> " << config.timeout_seconds << " s)" << std::endl;
        printState();
        std::cout << "============================\n" << std::endl;

        sleep(3);
        exit(0);
        return;
    }

    // ========================================================================
    // 速度控制计算（比例控制器）
    // 原理：误差越大，速度越快；误差越小，速度越慢
    // ========================================================================

    // 使用配置中的增益参数
    double gain_xy = config.gain_xy;   // 位置控制增益，值越大响应越快
    double gain_yaw = config.gain_yaw; // 朝向控制增益，值越大转向越快

    // 直接在机器人本体坐标系中计算速度（current_x/current_y 已经是本体坐标系）
    double vx = gain_xy * error_x;
    double vy = gain_xy * error_y;
    double vyaw = gain_yaw * error_yaw;

    // 限制速度上限（防止速度过大导致不稳定）
    double speed = std::sqrt(vx * vx + vy * vy);
    if (speed > config.max_speed)
    {
        vx = vx / speed * config.max_speed;
        vy = vy / speed * config.max_speed;
    }

    // 限制旋转速度上限
    if (std::abs(vyaw) > config.rotation_speed)
    {
        vyaw = vyaw / std::abs(vyaw) * config.rotation_speed;
    }

    // 发送速度指令
    // Move(vx, vy, vyaw): 控制机械狗以指定速度移动
    //   vx: 前后方向速度（m/s），正值为前进
    //   vy: 左右方向速度（m/s），正值为向左
    //   vyaw: 旋转速度（rad/s），正值为逆时针
    sport_client.Move(vx, vy, vyaw);
}

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, const char** argv)
{
    // 解析命令行参数
    NavigationConfig config;
    std::string network_interface;

    if (!parseArguments(argc, argv, config, network_interface))
    {
        return -1;  // 参数解析失败，已打印使用说明
    }

    // 初始化 DDS 通信
    ChannelFactory::Instance()->Init(0, network_interface.c_str());

    // 创建导航控制器并启动
    NavigationController nav;
    nav.Init(network_interface.c_str(), config);

    // 主线程保持运行
    while (1)
    {
        sleep(10);
    }

    return 0;
}
