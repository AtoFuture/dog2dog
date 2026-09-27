#include "go2_video_bridge/h264_stream_decoder.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
}

namespace go2_video_bridge
{

H264StreamDecoder::H264StreamDecoder(FrameCallback on_frame)
: on_frame_(std::move(on_frame))
{
  const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (codec == nullptr) {
    throw std::runtime_error("FFmpeg H.264 decoder is not available");
  }

  codec_context_ = avcodec_alloc_context3(codec);
  if (codec_context_ == nullptr) {
    throw std::runtime_error("avcodec_alloc_context3 failed");
  }

  const int open_result = avcodec_open2(codec_context_, codec, nullptr);
  if (open_result < 0) {
    avcodec_free_context(&codec_context_);
    throw std::runtime_error("avcodec_open2(H.264) failed");
  }

  parser_ = av_parser_init(AV_CODEC_ID_H264);
  if (parser_ == nullptr) {
    avcodec_free_context(&codec_context_);
    throw std::runtime_error("av_parser_init(H.264) failed");
  }

  frame_ = av_frame_alloc();
  if (frame_ == nullptr) {
    av_parser_close(parser_);
    parser_ = nullptr;
    avcodec_free_context(&codec_context_);
    throw std::runtime_error("av_frame_alloc failed");
  }
}

H264StreamDecoder::~H264StreamDecoder()
{
  av_frame_free(&frame_);
  if (parser_ != nullptr) {
    av_parser_close(parser_);
    parser_ = nullptr;
  }
  avcodec_free_context(&codec_context_);
}

void H264StreamDecoder::push(const std::uint8_t * data, std::size_t size)
{
  if (flushed_ || data == nullptr || size == 0) {
    return;
  }

  std::size_t offset = 0;
  while (offset < size) {
    const std::size_t remaining = size - offset;
    const int input_size = static_cast<int>(std::min<std::size_t>(
      remaining, static_cast<std::size_t>(std::numeric_limits<int>::max())));

    // av_parser_parse2 may read up to AV_INPUT_BUFFER_PADDING_SIZE bytes
    // past buf_size.  A ROS message vector is only sized to its payload, so
    // make a padded copy for every arbitrary input chunk.
    std::vector<std::uint8_t> padded_input(
      static_cast<std::size_t>(input_size) + AV_INPUT_BUFFER_PADDING_SIZE, 0);
    std::memcpy(padded_input.data(), data + offset, static_cast<std::size_t>(input_size));

    std::uint8_t * packet_data = nullptr;
    int packet_size = 0;
    const int consumed = av_parser_parse2(
      parser_,
      codec_context_,
      &packet_data,
      &packet_size,
      padded_input.data(),
      input_size,
      AV_NOPTS_VALUE,
      AV_NOPTS_VALUE,
      0);

    if (packet_size > 0) {
      ++packet_count_;
      decode_packet(packet_data, packet_size);
    }

    if (consumed < 0) {
      record_error(consumed);
      return;
    }
    if (consumed == 0) {
      // FFmpeg may emit a packet from bytes retained by the parser before it
      // consumes more of the current input.  That output must be handled
      // first; retrying the same input is then what the API's return contract
      // requires.  A zero/zero result cannot make progress and is reported.
      if (packet_size > 0) {
        continue;
      }
      record_error(AVERROR(EAGAIN));
      return;
    }

    offset += static_cast<std::size_t>(consumed);
  }
}

void H264StreamDecoder::flush()
{
  if (flushed_) {
    return;
  }
  flushed_ = true;

  // Let the parser emit a final access unit that it retained internally.
  std::uint8_t * packet_data = nullptr;
  int packet_size = 0;
  const int parser_result = av_parser_parse2(
    parser_,
    codec_context_,
    &packet_data,
    &packet_size,
    nullptr,
    0,
    AV_NOPTS_VALUE,
    AV_NOPTS_VALUE,
    0);
  if (parser_result < 0) {
    record_error(parser_result);
  } else if (packet_size > 0) {
    ++packet_count_;
    decode_packet(packet_data, packet_size);
  }

  const int send_result = avcodec_send_packet(codec_context_, nullptr);
  if (send_result < 0 && send_result != AVERROR_EOF && send_result != AVERROR(EAGAIN)) {
    record_error(send_result);
  }
  receive_frames();
}

void H264StreamDecoder::decode_packet(std::uint8_t * data, int size)
{
  AVPacket packet{};
  packet.data = data;
  packet.size = size;

  int send_result = avcodec_send_packet(codec_context_, &packet);
  if (send_result == AVERROR(EAGAIN)) {
    receive_frames();
    send_result = avcodec_send_packet(codec_context_, &packet);
  }
  if (send_result < 0 && send_result != AVERROR_EOF) {
    record_error(send_result);
    return;
  }
  receive_frames();
}

void H264StreamDecoder::receive_frames()
{
  while (true) {
    const int receive_result = avcodec_receive_frame(codec_context_, frame_);
    if (receive_result == 0) {
      ++decoded_frame_count_;
      if (on_frame_) {
        on_frame_(*frame_);
      }
      continue;
    }
    if (receive_result == AVERROR(EAGAIN) || receive_result == AVERROR_EOF) {
      return;
    }
    record_error(receive_result);
    return;
  }
}

void H264StreamDecoder::record_error(int error_code) noexcept
{
  (void)error_code;
  ++error_count_;
}

}  // namespace go2_video_bridge
