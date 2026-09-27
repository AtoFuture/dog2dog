#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

struct AVCodecContext;
struct AVCodecParserContext;
struct AVFrame;

namespace go2_video_bridge
{

/**
 * Incremental H.264 Annex-B decoder.
 *
 * The input to push() is an arbitrary byte chunk.  The parser owns any
 * incomplete NAL/access unit across calls; callers must not align chunks to
 * ROS messages, NAL units, or decoded frames.
 */
class H264StreamDecoder final
{
public:
  using FrameCallback = std::function<void(const AVFrame &)>;

  explicit H264StreamDecoder(FrameCallback on_frame);
  ~H264StreamDecoder();

  H264StreamDecoder(const H264StreamDecoder &) = delete;
  H264StreamDecoder & operator=(const H264StreamDecoder &) = delete;

  void push(const std::uint8_t * data, std::size_t size);

  // Only call this at end-of-stream.  A live ROS subscription must keep
  // feeding push() and must not flush the decoder between messages.
  void flush();

  std::uint64_t packet_count() const noexcept {return packet_count_;}
  std::uint64_t decoded_frame_count() const noexcept {return decoded_frame_count_;}
  std::uint64_t error_count() const noexcept {return error_count_;}

private:
  void decode_packet(std::uint8_t * data, int size);
  void receive_frames();
  void record_error(int error_code) noexcept;

  AVCodecContext * codec_context_{nullptr};
  AVCodecParserContext * parser_{nullptr};
  AVFrame * frame_{nullptr};
  FrameCallback on_frame_;
  std::uint64_t packet_count_{0};
  std::uint64_t decoded_frame_count_{0};
  std::uint64_t error_count_{0};
  bool flushed_{false};
};

}  // namespace go2_video_bridge
