/**********************************************************************
 Test wrapper for Unitree Go2 official sport posture example.
 Runs one selected TEST_MODE-style action for a bounded duration and exits.
***********************************************************************/

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

namespace
{

enum TestMode
{
  normal_stand,
  balance_stand,
  velocity_move,
  stand_down,
  stand_up,
  damp,
  recovery_stand,
  sit,
  rise_sit,
  stop_move = 99
};

void PrintUsage(const char* program)
{
  std::cout << "Usage: " << program << " networkInterface action [duration_ms]\n"
            << "Actions:\n"
            << "  normal_stand\n"
            << "  balance_stand\n"
            << "  stand_down\n"
            << "  stand_up\n"
            << "  damp\n"
            << "  recovery_stand\n"
            << "  sit\n"
            << "  rise_sit\n"
            << "  stop_move\n";
}

bool ParseMode(const std::string& action, TestMode& mode)
{
  if (action == "normal_stand") {
    mode = normal_stand;
  } else if (action == "balance_stand") {
    mode = balance_stand;
  } else if (action == "stand_down") {
    mode = stand_down;
  } else if (action == "stand_up") {
    mode = stand_up;
  } else if (action == "damp") {
    mode = damp;
  } else if (action == "recovery_stand") {
    mode = recovery_stand;
  } else if (action == "sit") {
    mode = sit;
  } else if (action == "rise_sit") {
    mode = rise_sit;
  } else if (action == "stop_move") {
    mode = stop_move;
  } else {
    return false;
  }
  return true;
}

std::string ModeText(uint8_t mode)
{
  switch (mode) {
  case 0:
    return "damp";
  case 1:
    return "sit_or_stand_down";
  case 2:
    return "stand";
  case 3:
  case 4:
  case 9:
  case 13:
    return "balance_stand";
  case 12:
    return "recovery_stand";
  default:
    return "unknown";
  }
}

class Custom
{
public:
  Custom()
  {
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
  }

  bool WaitForState(int timeout_ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      if (has_state.load()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  void PrintState(const std::string& title)
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    const auto& rpy = state.imu_state().rpy();
    const auto& position = state.position();
    std::cout << title
              << ": mode=" << static_cast<int>(state.mode())
              << " text=" << ModeText(state.mode())
              << " x=" << position[0]
              << " y=" << position[1]
              << " z=" << position[2]
              << " roll=" << rpy[0]
              << " pitch=" << rpy[1]
              << " yaw=" << rpy[2]
              << std::endl;
  }

  void SetMode(TestMode mode)
  {
    test_mode = mode;
    flag = 0;
    printed_first_ret = false;
  }

  void RobotControl()
  {
    int32_t ret = 0;
    switch (test_mode)
    {
    case normal_stand:
      ret = sport_client.StandUp();
      break;
    case balance_stand:
      ret = sport_client.BalanceStand();
      break;
    case velocity_move:
      break;
    case stand_down:
      ret = sport_client.StandDown();
      break;
    case stand_up:
      ret = sport_client.StandUp();
      break;
    case damp:
      ret = sport_client.Damp();
      break;
    case recovery_stand:
      ret = sport_client.RecoveryStand();
      break;
    case sit:
      if (flag == 0) {
        ret = sport_client.Sit();
        std::cout << "first_ret:" << ret << std::endl;
        flag = 1;
      }
      break;
    case rise_sit:
      if (flag == 0) {
        ret = sport_client.RiseSit();
        std::cout << "first_ret:" << ret << std::endl;
        flag = 1;
      }
      break;
    case stop_move:
      ret = sport_client.StopMove();
      break;
    default:
      ret = sport_client.StopMove();
    }
    if (!printed_first_ret && test_mode != sit && test_mode != rise_sit && test_mode != velocity_move) {
      std::cout << "first_ret:" << ret << std::endl;
      printed_first_ret = true;
    }
  }

  void HighStateHandler(const void* message)
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    state = *(unitree_go::msg::dds_::SportModeState_*)message;
    has_state.store(true);
  }

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
  std::atomic<bool> has_state{false};
  std::mutex state_mutex;
  TestMode test_mode = stop_move;
  int flag = 0;
  bool printed_first_ret = false;
  float dt = 0.005f;
};

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 3) {
    PrintUsage(argv[0]);
    return 2;
  }

  TestMode mode;
  const std::string action = argv[2];
  if (!ParseMode(action, mode)) {
    std::cout << "Unknown action: " << action << std::endl;
    PrintUsage(argv[0]);
    return 2;
  }

  int duration_ms = 1500;
  if (argc >= 4) {
    duration_ms = std::clamp(std::stoi(argv[3]), 200, 10000);
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  Custom custom;

  if (!custom.WaitForState(2000)) {
    std::cout << "No sport state received before action." << std::endl;
    return 1;
  }

  custom.PrintState("initial");
  custom.SetMode(mode);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    custom.RobotControl();
    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(custom.dt * 1000000)));
  }
  custom.PrintState("final");
  return 0;
}
