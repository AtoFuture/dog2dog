#include "go2_video_bridge/h264_stream_decoder.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

extern "C" {
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "unitree_go/msg/go2_front_video_data.hpp"

namespace go2_video_bridge
{

class Go2VideoBridgeNode final : public rclcpp::Node
{
public:
  Go2VideoBridgeNode()
  : Node("go2_video_bridge_node"),
    decoder_([this](const AVFrame & frame) {on_decoded_frame(frame);}),
    stats_started_at_(std::chrono::steady_clock::now())
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/frontvideostream");
    output_topic_ = declare_parameter<std::string>("output_topic", "/camera/go2/image_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    video_stream_ = declare_parameter<std::string>("video_stream", "video720p");

    if (video_stream_ != "video720p" &&
      video_stream_ != "video360p" && video_stream_ != "video180p")
    {
      RCLCPP_WARN(
        get_logger(), "unsupported video_stream='%s'; falling back to video720p",
        video_stream_.c_str());
      video_stream_ = "video720p";
    }

    if (frame_id_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "frame_id is empty; configure the real Go2 camera frame before 3-D localization");
    }

    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      output_topic_, rclcpp::SensorDataQoS());
    video_subscription_ = create_subscription<unitree_go::msg::Go2FrontVideoData>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&Go2VideoBridgeNode::on_video_message, this, std::placeholders::_1));

    stats_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&Go2VideoBridgeNode::log_stats, this));

    RCLCPP_INFO(
      get_logger(),
      "Go2 H.264 bridge: input=%s stream=%s output=%s QoS=SensorDataQoS",
      input_topic_.c_str(), video_stream_.c_str(), output_topic_.c_str());
  }

  ~Go2VideoBridgeNode() override
  {
    if (sws_context_ != nullptr) {
      sws_freeContext(sws_context_);
      sws_context_ = nullptr;
    }
  }

private:
  using Message = unitree_go::msg::Go2FrontVideoData;

  const auto * select_payload(const Message & message) const
  {
    if (video_stream_ == "video360p") {
      return &message.video360p;
    }
    if (video_stream_ == "video180p") {
      return &message.video180p;
    }
    return &message.video720p;
  }

  void on_video_message(const Message::ConstSharedPtr message)
  {
    const auto * payload = select_payload(*message);
    ++message_count_;
    payload_bytes_ += payload->size();
    last_time_frame_ = message->time_frame;

    if (!first_payload_logged_) {
      RCLCPP_INFO(
        get_logger(), "first %s payload length: %zu bytes",
        video_stream_.c_str(), payload->size());
      first_payload_logged_ = true;
    }

    decoder_.push(payload->data(), payload->size());
  }

  void on_decoded_frame(const AVFrame & frame)
  {
    ++decoded_frames_;
    last_width_ = frame.width;
    last_height_ = frame.height;

    if (!first_frame_logged_) {
      RCLCPP_INFO(
        get_logger(), "decoded first frame: %dx%d", frame.width, frame.height);
      first_frame_logged_ = true;
    }

    if (frame.width <= 0 || frame.height <= 0 || frame.data[0] == nullptr || frame.format < 0) {
      ++image_conversion_errors_;
      return;
    }

    sws_context_ = sws_getCachedContext(
      sws_context_,
      frame.width,
      frame.height,
      static_cast<AVPixelFormat>(frame.format),
      frame.width,
      frame.height,
      AV_PIX_FMT_BGR24,
      SWS_BILINEAR,
      nullptr,
      nullptr,
      nullptr);
    if (sws_context_ == nullptr) {
      ++image_conversion_errors_;
      return;
    }

    const int output_step = frame.width * 3;
    std::vector<std::uint8_t> bgr(
      static_cast<std::size_t>(output_step) * static_cast<std::size_t>(frame.height));
    std::uint8_t * destination_data[4] = {bgr.data(), nullptr, nullptr, nullptr};
    int destination_linesize[4] = {output_step, 0, 0, 0};
    const int output_rows = sws_scale(
      sws_context_,
      frame.data,
      frame.linesize,
      0,
      frame.height,
      destination_data,
      destination_linesize);
    if (output_rows != frame.height) {
      ++image_conversion_errors_;
      return;
    }

    auto image = std::make_unique<sensor_msgs::msg::Image>();
    // Go2FrontVideoData has no ROS Header.  This is deliberately the node
    // clock at decode/output time, not a conversion of time_frame.
    image->header.stamp = get_clock()->now();
    image->header.frame_id = frame_id_;
    image->height = static_cast<std::uint32_t>(frame.height);
    image->width = static_cast<std::uint32_t>(frame.width);
    image->encoding = "bgr8";
    image->is_bigendian = 0;
    image->step = static_cast<std::uint32_t>(output_step);
    image->data = std::move(bgr);
    image_publisher_->publish(std::move(image));
    ++published_frames_;
  }

  void log_stats()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - stats_started_at_).count();
    const double message_fps = elapsed > 0.0 ? static_cast<double>(message_count_) / elapsed : 0.0;
    const double frame_fps = elapsed > 0.0 ? static_cast<double>(published_frames_) / elapsed : 0.0;

    RCLCPP_INFO(
      get_logger(),
      "stats: messages=%llu payload_bytes=%llu packets=%llu decoded_frames=%llu "
      "published_frames=%llu decode_errors=%llu image_errors=%llu "
      "resolution=%dx%d input_msg_fps=%.2f output_fps=%.2f last_time_frame=%llu",
      static_cast<unsigned long long>(message_count_),
      static_cast<unsigned long long>(payload_bytes_),
      static_cast<unsigned long long>(decoder_.packet_count()),
      static_cast<unsigned long long>(decoded_frames_),
      static_cast<unsigned long long>(published_frames_),
      static_cast<unsigned long long>(decoder_.error_count()),
      static_cast<unsigned long long>(image_conversion_errors_),
      last_width_,
      last_height_,
      message_fps,
      frame_fps,
      static_cast<unsigned long long>(last_time_frame_));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  std::string video_stream_;

  H264StreamDecoder decoder_;
  SwsContext * sws_context_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Subscription<Message>::SharedPtr video_subscription_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::uint64_t message_count_{0};
  std::uint64_t payload_bytes_{0};
  std::uint64_t last_time_frame_{0};
  std::uint64_t decoded_frames_{0};
  std::uint64_t published_frames_{0};
  std::uint64_t image_conversion_errors_{0};
  int last_width_{0};
  int last_height_{0};
  bool first_payload_logged_{false};
  bool first_frame_logged_{false};
  std::chrono::steady_clock::time_point stats_started_at_;
};

}  // namespace go2_video_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_video_bridge::Go2VideoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
