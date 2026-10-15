/**********************************************************************
 GO2 SLAM精准导航程序
 功能：使用扩展坞上的SLAM服务实现精准导航到指定坐标
 注意：
   - 需要GO2 EDU型号 + 扩展坞 + 官方雷达（MID-360/XT16）
   - SLAM服务运行在扩展坞PC上（默认通过eth0连接192.168.123.x网段）
   - 使用前请确保SLAM服务已启动
 
 命令行参数说明：
   ./go2_slam_navigation <networkInterface> [选项]
   
   必需参数：
     networkInterface    网卡名称（如 eth0，需连接192.168.123.x网段）
   
   可选参数（未指定时使用默认值）：
     --target-x <value>      目标 x 坐标（米），默认 1.0
     --target-y <value>      目标 y 坐标（米），默认 0.0
     --target-yaw <value>    目标朝向（弧度），默认 0.0
     --timeout <value>       超时时间（秒），默认 60.0
   
   示例：
     # 前进 1 米
     ./go2_slam_navigation eth0
     
     # 前进 2 米，左转 90 度
     ./go2_slam_navigation eth0 --target-x 2.0 --target-yaw 1.57
     
     # 后退 1 米
     ./go2_slam_navigation eth0 --target-x -1.0
**********************************************************************/

#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/String_.hpp>

// SLAM话题定义
#define TOPIC_SLAM_INFO "rt/slam_info"

// ============================================================================
// 导航配置结构体
// ============================================================================
struct NavigationConfig
{
    double target_x = 1.0;       // 目标 x 坐标（米）
    double target_y = 0.0;       // 目标 y 坐标（米）
    double target_yaw = 0.0;     // 目标朝向（弧度）
    double timeout_seconds = 60.0; // 超时时间（秒）
};

// ============================================================================
// SLAM导航控制器类
// ============================================================================
class SLAMNavigationController
{
public:
    SLAMNavigationController() : slam_info_valid(false), current_x(0), current_y(0), current_yaw(0) {}
    ~SLAMNavigationController() {}

    // 初始化函数
    void Init(const char* network_interface, const NavigationConfig& cfg);

    // 执行导航
    void Navigate();

    // 打印当前状态
    void printState();

private:
    // SLAM信息回调
    void SlamInfoHandler(const void* message);

    // 从JSON字符串中提取数值
    double extractJsonValue(const std::string& json, const std::string& key);

    // 计算角度差
    double calcAngleDiff(double angle1, double angle2);

    // 计算距离
    double calcDistance(double x1, double y1, double x2, double y2);

    unitree::robot::go2::SportClient sport_client;  // 运动控制客户端

    std::shared_ptr<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>> slam_info_suber;

    bool slam_info_valid;
    double current_x, current_y, current_yaw;

    NavigationConfig config;
};

// ============================================================================
// 初始化函数
// ============================================================================
void SLAMNavigationController::Init(const char* network_interface, const NavigationConfig& cfg)
{
    config = cfg;

    std::cout << "\n[DEBUG] 接收到的配置:" << std::endl;
    std::cout << "  target_x=" << config.target_x << std::endl;
    std::cout << "  target_y=" << config.target_y << std::endl;
    std::cout << "  target_yaw=" << config.target_yaw << std::endl;
    std::cout << "  timeout=" << config.timeout_seconds << std::endl;

    // 初始化运动客户端
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    // 订阅SLAM信息话题
    slam_info_suber.reset(new unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>(TOPIC_SLAM_INFO));
    slam_info_suber->InitChannel(std::bind(&SLAMNavigationController::SlamInfoHandler, this, std::placeholders::_1), 1);

    // 等待SLAM状态数据
    std::cout << "等待SLAM状态数据..." << std::endl;
    int wait_count = 0;
    while (wait_count < 300 && !slam_info_valid)
    {
        usleep(10000);
        wait_count++;
    }

    if (slam_info_valid)
    {
        std::cout << "\n========== GO2 SLAM精准导航 ==========" << std::endl;
        std::cout << "当前位置: x=" << current_x << ", y=" << current_y << ", yaw=" << current_yaw << std::endl;
        std::cout << "目标位置: x=" << config.target_x << ", y=" << config.target_y << ", yaw=" << config.target_yaw << std::endl;
        std::cout << "超时时间: " << config.timeout_seconds << " s" << std::endl;
        std::cout << "======================================\n" << std::endl;
    }
    else
    {
        std::cout << "警告: 无法获取SLAM状态数据，将使用基础导航模式" << std::endl;
    }

    // 启动平衡站立模式
    std::cout << "[DEBUG] 调用 BalanceStand..." << std::endl;
    int ret = sport_client.BalanceStand();
    std::cout << "[DEBUG] BalanceStand 返回: " << ret << std::endl;
    sleep(1);
}

// ============================================================================
// SLAM信息回调
// ============================================================================
void SLAMNavigationController::SlamInfoHandler(const void* message)
{
    auto msg = *(std_msgs::msg::dds_::String_*)message;
    std::string data(msg.data().begin(), msg.data().end());
    
    // [DEBUG] 打印收到的原始SLAM数据
    static int msg_count = 0;
    msg_count++;
    if (msg_count <= 3)
    {
        std::cout << "\n[DEBUG-SLAM] 收到第 " << msg_count << " 条消息，长度: " << data.length() << std::endl;
        std::cout << "[DEBUG-SLAM] 原始数据: " << data.substr(0, 500) << std::endl;
        if (data.length() > 500)
            std::cout << "[DEBUG-SLAM] ... (截断)" << std::endl;
    }
    
    // 简单解析JSON格式的SLAM数据
    // 格式: {"errorCode":0, "data":{"position":{"x":1.0,"y":2.0,"yaw":0.5},...}}
    double x = extractJsonValue(data, "\"x\"");
    double y = extractJsonValue(data, "\"y\"");
    double yaw = extractJsonValue(data, "\"yaw\"");
    
    // [DEBUG] 打印提取的值
    if (msg_count <= 3)
    {
        std::cout << "[DEBUG-SLAM] 提取值: x=" << x << ", y=" << y << ", yaw=" << yaw << std::endl;
        std::cout << "[DEBUG-SLAM] 包含position: " << (data.find("\"position\"") != std::string::npos ? "是" : "否") << std::endl;
    }
    
    // 检查是否获取到有效数据
    if (data.find("\"position\"") != std::string::npos && 
        data.find("\"x\"") != std::string::npos)
    {
        current_x = x;
        current_y = y;
        current_yaw = yaw;
        slam_info_valid = true;
        if (msg_count <= 3)
        {
            std::cout << "[DEBUG-SLAM] 状态标记为有效!" << std::endl;
        }
    }
    else
    {
        if (msg_count <= 3)
        {
            std::cout << "[DEBUG-SLAM] 状态标记为无效 (缺少position或x字段)" << std::endl;
        }
    }
}

// ============================================================================
// 从JSON字符串中提取数值
// ============================================================================
double SLAMNavigationController::extractJsonValue(const std::string& json, const std::string& key)
{
    size_t key_pos = json.find(key);
    if (key_pos == std::string::npos) return 0.0;
    
    size_t colon_pos = json.find(':', key_pos + key.length());
    if (colon_pos == std::string::npos) return 0.0;
    
    size_t value_start = colon_pos + 1;
    // 跳过空格
    while (value_start < json.length() && (json[value_start] == ' ' || json[value_start] == '\t'))
        value_start++;
    
    if (value_start >= json.length()) return 0.0;
    
    // 提取数值字符串
    size_t value_end = value_start;
    while (value_end < json.length() && 
           (isdigit(json[value_end]) || json[value_end] == '.' || json[value_end] == '-'))
        value_end++;
    
    if (value_end == value_start) return 0.0;
    
    std::string value_str = json.substr(value_start, value_end - value_start);
    try
    {
        return std::stod(value_str);
    }
    catch (...)
    {
        return 0.0;
    }
}

// ============================================================================
// 执行导航
// ============================================================================
void SLAMNavigationController::Navigate()
{
    auto start_time = std::chrono::steady_clock::now();
    int check_count = 0;

    std::cout << "\n开始导航..." << std::endl;

    while (true)
    {
        // 检查超时
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        if (elapsed > config.timeout_seconds)
        {
            std::cout << "\n========== 超时 ==========" << std::endl;
            std::cout << "运行时间: " << elapsed << " s (> " << config.timeout_seconds << " s)" << std::endl;
            sport_client.StopMove();
            sleep(3);
            return;
        }

        // 检查位置数据是否有效
        if (!slam_info_valid)
        {
            std::cout << "警告: SLAM位置数据无效，跳过本次检查" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // 计算误差
        double error_x = config.target_x - current_x;
        double error_y = config.target_y - current_y;
        double error_yaw = calcAngleDiff(current_yaw, config.target_yaw);
        double distance_to_target = calcDistance(current_x, current_y, config.target_x, config.target_y);

        // 打印进度（每500ms一次）
        check_count++;
        if (check_count % 5 == 0)
        {
            std::cout << "距离目标: " << distance_to_target << " m | 朝向误差: " << (error_yaw * 180.0 / M_PI) << "°" << std::endl;
        }

        // 判定是否到达目标点
        if (distance_to_target < 0.15 && std::abs(error_yaw) < 0.1)
        {
            std::cout << "\n========== 到达目标点 ==========" << std::endl;
            printState();
            std::cout << "\n位置误差: " << distance_to_target << " m (< 0.15 m)" << std::endl;
            std::cout << "朝向误差: " << (error_yaw * 180.0 / M_PI) << "° (< 5.7°)" << std::endl;
            std::cout << "================================\n" << std::endl;
            sport_client.StopMove();
            sleep(3);
            return;
        }

        // 比例控制计算速度
        double vx = 1.5 * error_x;
        double vy = 1.5 * error_y;
        double vyaw = 1.2 * error_yaw;

        // 限制速度
        double speed = std::sqrt(vx * vx + vy * vy);
        if (speed > 0.5)
        {
            vx = vx / speed * 0.5;
            vy = vy / speed * 0.5;
        }
        if (std::abs(vyaw) > 1.0)
        {
            vyaw = vyaw / std::abs(vyaw) * 1.0;
        }

        sport_client.Move(vx, vy, vyaw);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ============================================================================
// 打印状态
// ============================================================================
void SLAMNavigationController::printState()
{
    if (slam_info_valid)
    {
        std::cout << "当前位置: x=" << current_x << ", y=" << current_y << ", yaw=" << current_yaw << std::endl;
    }
    else
    {
        std::cout << "无法获取当前位置" << std::endl;
    }
}

// ============================================================================
// 工具函数
// ============================================================================
double SLAMNavigationController::calcAngleDiff(double angle1, double angle2)
{
    double diff = angle2 - angle1;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

double SLAMNavigationController::calcDistance(double x1, double y1, double x2, double y2)
{
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "用法: " << argv[0] << " <networkInterface> [选项]" << std::endl;
        std::cout << "  networkInterface: 网卡名称（如 eth0，需连接192.168.123.x网段）" << std::endl;
        std::cout << "\n可选参数:" << std::endl;
        std::cout << "  --target-x <value>      目标 x 坐标（米），默认 1.0" << std::endl;
        std::cout << "  --target-y <value>      目标 y 坐标（米），默认 0.0" << std::endl;
        std::cout << "  --target-yaw <value>    目标朝向（弧度），默认 0.0" << std::endl;
        std::cout << "  --timeout <value>       超时时间（秒），默认 60.0" << std::endl;
        exit(-1);
    }

    const char* network_interface = argv[1];
    NavigationConfig config;

    // 解析命令行参数
    for (int i = 2; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--target-x" && i + 1 < argc)
        {
            config.target_x = std::stod(argv[++i]);
        }
        else if (arg == "--target-y" && i + 1 < argc)
        {
            config.target_y = std::stod(argv[++i]);
        }
        else if (arg == "--target-yaw" && i + 1 < argc)
        {
            config.target_yaw = std::stod(argv[++i]);
        }
        else if (arg == "--timeout" && i + 1 < argc)
        {
            config.timeout_seconds = std::stod(argv[++i]);
        }
    }

    // 初始化DDS
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    SLAMNavigationController controller;
    controller.Init(network_interface, config);
    controller.Navigate();

    return 0;
}
