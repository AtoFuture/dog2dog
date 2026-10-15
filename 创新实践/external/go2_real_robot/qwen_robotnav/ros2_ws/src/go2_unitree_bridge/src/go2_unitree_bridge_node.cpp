/**********************************************************************
 * Standalone Unitree SDK2 bridge.
 * - Receives velocity commands over UDP (cmd_vel port) and forwards
 *   them to the Go2 SportClient at high rate.
 * - Grabs camera frames from VideoClient and streams JPEG over UDP.
 * This process does NOT link ROS2, avoiding DDS conflicts.
 **********************************************************************/

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

using namespace std::chrono_literals;
using unitree::robot::go2::SportClient;
using unitree::robot::go2::VideoClient;

static constexpr int kCmdPort = 5000;
static constexpr int kImagePort = 5001;
static constexpr size_t kMaxUdpPacket = 65507;

struct CmdVel {
  float vx{0.0f};
  float vy{0.0f};
  float vyaw{0.0f};
  std::chrono::steady_clock::time_point received;
};

class Go2UnitreeBridge {
 public:
  explicit Go2UnitreeBridge(const std::string &iface) : iface_(iface) {}

  bool Init(bool stand_up = true) {
    unitree::robot::ChannelFactory::Instance()->Init(
        0, const_cast<char *>(iface_.c_str()));

    // Construct SDK clients AFTER ChannelFactory::Init to avoid DDS
    // participant initialisation races.
    sport_client_.reset(new SportClient());
    video_client_.reset(new VideoClient());

    sport_client_->SetTimeout(10.0f);
    sport_client_->Init();

    video_client_->SetTimeout(1.0f);
    video_client_->Init();

    if (stand_up) {
      printf("Standing up Go2...\n");
      sport_client_->StandUp();
      std::this_thread::sleep_for(800ms);
      sport_client_->BalanceStand();
      std::this_thread::sleep_for(200ms);
    } else {
      printf("Skipping stand-up (--no-stand).\n");
    }
    fflush(stdout);

    cmd_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (cmd_sock_ < 0) {
      perror("cmd socket");
      return false;
    }
    int reuse = 1;
    setsockopt(cmd_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in cmd_addr{};
    cmd_addr.sin_family = AF_INET;
    cmd_addr.sin_addr.s_addr = INADDR_ANY;
    cmd_addr.sin_port = htons(kCmdPort);
    if (bind(cmd_sock_, reinterpret_cast<sockaddr *>(&cmd_addr),
             sizeof(cmd_addr)) < 0) {
      perror("cmd bind");
      return false;
    }

    image_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (image_sock_ < 0) {
      perror("image socket");
      return false;
    }

    return true;
  }

  void Run() {
    stop_ = false;
    cmd_thread_ = std::thread(&Go2UnitreeBridge::CmdReceiveLoop, this);
    control_thread_ = std::thread(&Go2UnitreeBridge::ControlLoop, this);
    video_thread_ = std::thread(&Go2UnitreeBridge::VideoLoop, this);

    cmd_thread_.join();
    control_thread_.join();
    video_thread_.join();
  }

  void Stop() {
    stop_ = true;
    if (cmd_sock_ >= 0) shutdown(cmd_sock_, SHUT_RDWR);
    if (image_sock_ >= 0) shutdown(image_sock_, SHUT_RDWR);
  }

  ~Go2UnitreeBridge() {
    Stop();
    if (cmd_thread_.joinable()) cmd_thread_.join();
    if (control_thread_.joinable()) control_thread_.join();
    if (video_thread_.joinable()) video_thread_.join();
    if (cmd_sock_ >= 0) close(cmd_sock_);
    if (image_sock_ >= 0) close(image_sock_);
    try {
      if (sport_client_ && motion_active_) sport_client_->StopMove();
    } catch (...) {
    }
  }

 private:
  void CmdReceiveLoop() {
    char buf[256];
    sockaddr_in sender{};
    socklen_t sender_len = sizeof(sender);
    while (!stop_) {
      ssize_t n = recvfrom(cmd_sock_, buf, sizeof(buf) - 1, 0,
                           reinterpret_cast<sockaddr *>(&sender), &sender_len);
      if (n < 0) {
        if (errno == EINTR || errno == EAGAIN) continue;
        break;
      }
      buf[n] = '\0';
      CmdVel cmd;
      if (!ParseCmd(buf, &cmd)) {
        fprintf(stderr, "Ignoring invalid cmd_vel packet: %s\n", buf);
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        latest_cmd_ = cmd;
        has_cmd_ = true;
      }
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        ros_target_addr_ = sender;
        // Commands originate from an ephemeral UDP port, while the ROS bridge
        // listens for camera packets on the fixed image port.
        ros_target_addr_.sin_port = htons(kImagePort);
        ros_target_addr_set_ = true;
      }
    }
  }

  // Keep parsing deliberately small because this bridge has no JSON library,
  // but advance past ':' before strtod.  Starting at ':' silently decodes
  // every otherwise-valid velocity as zero.
  static bool ParseNumber(const char *json, const char *key, float *value) {
    const char *p = strstr(json, key);
    if (!p) return false;
    p = strchr(p + strlen(key), ':');
    if (!p) return false;

    errno = 0;
    char *end = nullptr;
    double parsed = strtod(p + 1, &end);
    if (end == p + 1 || errno == ERANGE || !std::isfinite(parsed)) return false;
    *value = static_cast<float>(parsed);
    return true;
  }

  static bool ParseCmd(const char *json, CmdVel *cmd) {
    if (!json || !cmd) return false;

    CmdVel parsed;
    if (!ParseNumber(json, "\"vx\"", &parsed.vx) ||
        !ParseNumber(json, "\"vy\"", &parsed.vy) ||
        !ParseNumber(json, "\"vyaw\"", &parsed.vyaw)) {
      return false;
    }
    parsed.received = std::chrono::steady_clock::now();
    *cmd = parsed;
    return true;
  }

  void ControlLoop() {
    while (!stop_) {
      auto loop_start = std::chrono::steady_clock::now();

      CmdVel cmd;
      bool has_cmd = false;
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd = latest_cmd_;
        has_cmd = has_cmd_;
      }

      if (has_cmd) {
        bool stale = std::chrono::duration_cast<std::chrono::milliseconds>(
                         loop_start - cmd.received)
                             .count() > 500;
        bool zero = std::abs(cmd.vx) < 1e-4f && std::abs(cmd.vy) < 1e-4f &&
                    std::abs(cmd.vyaw) < 1e-4f;

        if (stale || zero) {
          // Stop once when this bridge relinquishes control.  Repeating
          // StopMove at 100 Hz would starve other valid SportClient users.
          if (motion_active_) {
            sport_client_->StopMove();
            motion_active_ = false;
          }
        } else {
          sport_client_->Move(cmd.vx, cmd.vy, cmd.vyaw);
          motion_active_ = true;
        }
      }

      auto elapsed = std::chrono::steady_clock::now() - loop_start;
      auto sleep = 10ms - elapsed;
      if (sleep > 0ms) std::this_thread::sleep_for(sleep);
    }
  }

  void VideoLoop() {
    while (!stop_) {
      std::vector<uint8_t> jpeg;
      int ret = video_client_->GetImageSample(jpeg);
      if (ret != 0 || jpeg.empty()) {
        std::this_thread::sleep_for(50ms);
        continue;
      }

      sockaddr_in target;
      bool has_target = false;
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (ros_target_addr_set_) {
          target = ros_target_addr_;
          has_target = true;
        }
      }

      if (has_target) {
        // Send length-prefixed chunks.
        uint32_t total = htonl(static_cast<uint32_t>(jpeg.size()));
        sendto(image_sock_, &total, sizeof(total), 0,
               reinterpret_cast<sockaddr *>(&target), sizeof(target));

        size_t offset = 0;
        while (offset < jpeg.size()) {
          size_t chunk = std::min(jpeg.size() - offset, kMaxUdpPacket);
          sendto(image_sock_, jpeg.data() + offset, chunk, 0,
                 reinterpret_cast<sockaddr *>(&target), sizeof(target));
          offset += chunk;
        }
      }

      std::this_thread::sleep_for(100ms);  // ~10 fps
    }
  }

  std::string iface_;
  std::unique_ptr<SportClient> sport_client_;
  std::unique_ptr<VideoClient> video_client_;

  int cmd_sock_{-1};
  int image_sock_{-1};

  std::atomic<bool> stop_{false};
  std::thread cmd_thread_;
  std::thread control_thread_;
  std::thread video_thread_;

  std::mutex cmd_mutex_;
  CmdVel latest_cmd_;
  bool has_cmd_{false};
  bool motion_active_{false};

  std::mutex target_mutex_;
  sockaddr_in ros_target_addr_{};
  bool ros_target_addr_set_{false};
};

int main(int argc, char **argv) {
  std::string iface = "eth0";
  bool stand_up = true;
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--no-stand") {
      stand_up = false;
    } else if (arg == "--help" || arg == "-h") {
      printf("Usage: %s [interface] [--no-stand]\n", argv[0]);
      printf("  interface   Network interface for Unitree SDK2 (default: eth0)\n");
      printf("  --no-stand  Do not stand up the robot on start\n");
      return 0;
    } else if (arg.rfind("--", 0) == 0) {
      fprintf(stderr, "Unknown option: %s\n", arg.c_str());
      return 1;
    } else {
      iface = arg;
    }
  }

  Go2UnitreeBridge bridge(iface);
  if (!bridge.Init(stand_up)) {
    fprintf(stderr, "Failed to initialise bridge\n");
    return 1;
  }

  printf("go2_unitree_bridge running. UDP cmd port=%d, image port=%d\n",
         kCmdPort, kImagePort);
  fflush(stdout);
  bridge.Run();
  return 0;
}
