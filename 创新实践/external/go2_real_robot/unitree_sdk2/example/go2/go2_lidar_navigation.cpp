/**********************************************************************
 GO2 激光雷达闭环导航程序
 功能：使用GO2内置激光雷达避障 + 里程计反馈实现闭环导航
 注意：
   - 需要GO2 EDU型号 + 内置激光雷达
   - 激光雷达服务开机自启，通过APP可开启/关闭
   - 使用比例控制器 + 障碍物距离检测实现简单避障
   - 订阅 rt/sportmodestate 获取机器人位置反馈
 
 命令行参数说明：
   ./go2_lidar_navigation <networkInterface> [选项]
   
   必需参数：
     networkInterface    网卡名称（如 eth0）
   
   可选参数（未指定时使用默认值）：
     --target-x <value>      目标 x 坐标（米），默认 1.0
     --target-y <value>      目标 y 坐标（米），默认 0.0
     --target-yaw <value>    目标朝向（弧度），默认 0.0
     --timeout <value>       超时时间（秒），默认 60.0
     --max-speed <value>     最大速度（m/s），默认 0.3
     --obstacle-dist <value> 障碍物安全距离（米），默认 0.5
   
   示例：
     # 前进 1 米
     ./go2_lidar_navigation eth0
     
     # 前进 2 米，左转 90 度
     ./go2_lidar_navigation eth0 --target-x 2.0 --target-yaw 1.57
     
     # 后退 1 米
     ./go2_lidar_navigation eth0 --target-x -1.0
**********************************************************************/

#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/PointStamped_.hpp>

// 话题定义
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_RANGE_INFO "rt/utlidar/range_info"

// ============================================================================
// 导航配置结构体
// ============================================================================
struct NavigationConfig
{
    double target_x = 1.0;       // 目标 x 坐标（米）
    double target_y = 0.0;       // 目标 y 坐标（米）
    double target_yaw = 0.0;     // 目标朝向（弧度）
    double timeout_seconds = 60.0; // 超时时间（秒）
    double max_speed = 0.3;      // 最大速度（m/s）- 降低到0.3确保安全
    double obstacle_dist = 0.5;  // 障碍物安全距离（米）
};

// ============================================================================
// 机器人状态结构体
// ============================================================================
struct RobotState
{
    double x = 0;          // 当前位置 x
    double y = 0;          // 当前位置 y
    double yaw = 0;        // 当前朝向
    bool valid = false;    // 数据是否有效
};

// ============================================================================
// 障碍物信息结构体
// ============================================================================
struct ObstacleInfo
{
    double front = 999.0;  // 前方障碍物距离
    double left = 999.0;   // 左方障碍物距离
    double right = 999.0;  // 右方障碍物距离
    bool valid = false;    // 数据是否有效
};

// ============================================================================
// 激光雷达导航控制器类
// ============================================================================
class LidarNavigationController
{
public:
    LidarNavigationController() : init_yaw_set(false) {}
    ~LidarNavigationController() {}

    // 初始化函数
    void Init(const char* network_interface, const NavigationConfig& cfg);

    // 执行导航
    void Navigate();

private:
    // 机器人状态回调
    void HighStateHandler(const void* message);
    
    // 障碍物信息回调
    void RangeInfoHandler(const void* message);

    // 计算角度差
    double calcAngleDiff(double angle1, double angle2);

    // 计算距离
    double calcDistance(double x1, double y1, double x2, double y2);

    unitree::robot::go2::SportClient sport_client;  // 运动控制客户端

    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> state_suber;
    std::shared_ptr<unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_>> range_suber;

    RobotState robot;
    ObstacleInfo obstacle;
    std::mutex obstacle_mutex;  // 保护障碍物数据
    NavigationConfig config;

    // 初始状态
    double init_x, init_y, init_yaw;
    bool init_yaw_set;
};

// ============================================================================
// 初始化函数
// ============================================================================
void LidarNavigationController::Init(const char* network_interface, const NavigationConfig& cfg)
{
    config = cfg;

    std::cout << "\n========== GO2 激光雷达闭环导航 ==========" << std::endl;
    std::cout << "目标位置: x=" << config.target_x << ", y=" << config.target_y << ", yaw=" << config.target_yaw << std::endl;
    std::cout << "最大速度: " << config.max_speed << " m/s" << std::endl;
    std::cout << "障碍物安全距离: " << config.obstacle_dist << " m" << std::endl;
    std::cout << "超时时间: " << config.timeout_seconds << " s" << std::endl;
    std::cout << "======================================\n" << std::endl;

    // 初始化运动客户端
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    // 订阅机器人状态话题（获取位置）
    state_suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    state_suber->InitChannel(std::bind(&LidarNavigationController::HighStateHandler, this, std::placeholders::_1), 1);

    // 订阅激光雷达障碍物信息话题
    range_suber.reset(new unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_>(TOPIC_RANGE_INFO));
    range_suber->InitChannel(std::bind(&LidarNavigationController::RangeInfoHandler, this, std::placeholders::_1), 1);

    // 等待机器人状态数据
    std::cout << "等待机器人状态数据..." << std::endl;
    int wait_count = 0;
    while (wait_count < 300 && !robot.valid)
    {
        usleep(10000);
        wait_count++;
    }

    if (robot.valid)
    {
        std::cout << "机器人状态已就绪" << std::endl;
        std::cout << "  初始位置: x=" << robot.x << ", y=" << robot.y << ", yaw=" << (robot.yaw * 180.0 / M_PI) << "°" << std::endl;
    }
    else
    {
        std::cout << "错误: 无法获取机器人状态数据" << std::endl;
        exit(-1);
    }

    // 等待障碍物数据
    std::cout << "等待激光雷达数据..." << std::endl;
    wait_count = 0;
    while (wait_count < 300 && !obstacle.valid)
    {
        usleep(10000);
        wait_count++;
    }

    if (obstacle.valid)
    {
        std::cout << "激光雷达数据已就绪" << std::endl;
        std::cout << "  前方障碍物: " << obstacle.front << " m" << std::endl;
        std::cout << "  左方障碍物: " << obstacle.left << " m" << std::endl;
        std::cout << "  右方障碍物: " << obstacle.right << " m" << std::endl;
    }
    else
    {
        std::cout << "警告: 无法获取激光雷达数据，将使用无避障模式" << std::endl;
    }

    // 记录初始位置为原点
    init_x = robot.x;
    init_y = robot.y;
    init_yaw = robot.yaw;
    init_yaw_set = true;

    std::cout << "\n初始位置设为原点: (0, 0, " << (init_yaw * 180.0 / M_PI) << "°)" << std::endl;

    // 启动平衡站立模式
    std::cout << "\n启动平衡站立模式..." << std::endl;
    int ret = sport_client.BalanceStand();
    std::cout << "BalanceStand 返回: " << ret << std::endl;
    sleep(1);
}

// ============================================================================
// 机器人状态回调
// ============================================================================
void LidarNavigationController::HighStateHandler(const void* message)
{
    auto state = *(unitree_go::msg::dds_::SportModeState_*)message;
    
    robot.x = state.position()[0];
    robot.y = state.position()[1];
    robot.yaw = state.imu_state().rpy()[2];
    robot.valid = true;
}

// ============================================================================
// 障碍物信息回调
// ============================================================================
void LidarNavigationController::RangeInfoHandler(const void* message)
{
    auto msg = *(geometry_msgs::msg::dds_::PointStamped_*)message;
    
    std::lock_guard<std::mutex> lock(obstacle_mutex);
    obstacle.front = msg.point().x();
    obstacle.left = msg.point().y();
    obstacle.right = msg.point().z();
    obstacle.valid = true;
}

// ============================================================================
// 执行导航
// ============================================================================
void LidarNavigationController::Navigate()
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

        // 计算相对于初始位置的当前位置
        double dx = robot.x - init_x;
        double dy = robot.y - init_y;
        double dyaw = robot.yaw - init_yaw;

        // 转换到初始朝向定义的局部坐标系
        double cos_init = cos(init_yaw);
        double sin_init = sin(init_yaw);
        double current_x = cos_init * dx + sin_init * dy;
        double current_y = -sin_init * dx + cos_init * dy;
        double current_yaw = dyaw;

        // 计算误差
        double error_x = config.target_x - current_x;
        double error_y = config.target_y - current_y;
        double error_yaw = calcAngleDiff(current_yaw, config.target_yaw);
        double distance_to_target = calcDistance(current_x, current_y, config.target_x, config.target_y);

        // 打印进度（每500ms一次）
        check_count++;
        if (check_count % 5 == 0)
        {
            std::cout << "当前位置: (" << current_x << ", " << current_y << ") | 距离目标: " << distance_to_target << " m" << std::endl;
            if (obstacle.valid)
            {
                std::cout << "  障碍物: 前=" << obstacle.front << "m, 左=" << obstacle.left << "m, 右=" << obstacle.right << "m" << std::endl;
            }
        }

        // 判定是否到达目标点
        if (distance_to_target < 0.1 && std::abs(error_yaw) < 0.1)
        {
            std::cout << "\n========== 到达目标点 ==========" << std::endl;
            std::cout << "位置误差: " << distance_to_target << " m (< 0.1 m)" << std::endl;
            std::cout << "朝向误差: " << (error_yaw * 180.0 / M_PI) << "° (< 5.7°)" << std::endl;
            std::cout << "================================\n" << std::endl;
            sport_client.StopMove();
            sleep(3);
            return;
        }

        // 障碍物检测与避障（使用局部副本避免竞态条件）
        double vx = 0, vy = 0, vyaw = 0;
        
        ObstacleInfo local_obstacle;
        {
            std::lock_guard<std::mutex> lock(obstacle_mutex);
            local_obstacle = obstacle;
        }
        
        if (local_obstacle.valid)
        {
            // 前方障碍物检测（只在实际检测到障碍物且距离过近时避障）
            // 注意：inf 表示前方无障碍物，不应触发避障
            if (std::isfinite(local_obstacle.front) && local_obstacle.front < config.obstacle_dist)
            {
                std::cout << "[避障] 前方障碍物过近 (" << local_obstacle.front << "m)，停止前进" << std::endl;
                vx = 0;
                vy = 0;
                // 尝试转向避开
                if (std::isfinite(local_obstacle.left) && std::isfinite(local_obstacle.right))
                {
                    if (local_obstacle.left > local_obstacle.right)
                        vyaw = 0.5;  // 左转
                    else
                        vyaw = -0.5; // 右转
                }
                else
                {
                    vyaw = 0.5;  // 默认左转
                }
            }
            else
            {
                // 正常导航：比例控制计算速度
                vx = 1.5 * error_x;
                vy = 1.5 * error_y;
                vyaw = 1.2 * error_yaw;

                // 限制速度
                double speed = std::sqrt(vx * vx + vy * vy);
                if (speed > config.max_speed)
                {
                    vx = vx / speed * config.max_speed;
                    vy = vy / speed * config.max_speed;
                }
                if (std::abs(vyaw) > 1.0)
                {
                    vyaw = vyaw / std::abs(vyaw) * 1.0;
                }
            }
        }
        else
        {
            // 无激光雷达数据：直接导航
            vx = 1.5 * error_x;
            vy = 1.5 * error_y;
            vyaw = 1.2 * error_yaw;

            double speed = std::sqrt(vx * vx + vy * vy);
            if (speed > config.max_speed)
            {
                vx = vx / speed * config.max_speed;
                vy = vy / speed * config.max_speed;
            }
            if (std::abs(vyaw) > 1.0)
            {
                vyaw = vyaw / std::abs(vyaw) * 1.0;
            }
        }

        sport_client.Move(vx, vy, vyaw);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ============================================================================
// 工具函数
// ============================================================================
double LidarNavigationController::calcAngleDiff(double angle1, double angle2)
{
    double diff = angle2 - angle1;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

double LidarNavigationController::calcDistance(double x1, double y1, double x2, double y2)
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
        std::cout << "  networkInterface: 网卡名称（如 eth0）" << std::endl;
        std::cout << "\n可选参数:" << std::endl;
        std::cout << "  --target-x <value>      目标 x 坐标（米），默认 1.0" << std::endl;
        std::cout << "  --target-y <value>      目标 y 坐标（米），默认 0.0" << std::endl;
        std::cout << "  --target-yaw <value>    目标朝向（弧度），默认 0.0" << std::endl;
        std::cout << "  --timeout <value>       超时时间（秒），默认 60.0" << std::endl;
        std::cout << "  --max-speed <value>     最大速度（m/s），默认 0.3" << std::endl;
        std::cout << "  --obstacle-dist <value> 障碍物安全距离（米），默认 0.5" << std::endl;
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
        else if (arg == "--max-speed" && i + 1 < argc)
        {
            config.max_speed = std::stod(argv[++i]);
        }
        else if (arg == "--obstacle-dist" && i + 1 < argc)
        {
            config.obstacle_dist = std::stod(argv[++i]);
        }
    }

    // 初始化DDS
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    LidarNavigationController controller;
    controller.Init(network_interface, config);
    controller.Navigate();

    return 0;
}
