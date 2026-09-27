#include "go2_video_bridge/h264_stream_decoder.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <random>
#include <string>
#include <utility>
#include <vector>

extern "C" {
#include <libavutil/imgutils.h>
#include <libavutil/pixfmt.h>
#include <libavutil/frame.h>
}

namespace
{

struct RunResult
{
  std::size_t frames{0};
  std::size_t packets{0};
  std::size_t errors{0};
  int width{0};
  int height{0};
  bool valid_frames{true};
};

RunResult run_with_chunks(const std::vector<std::uint8_t> & bytes, std::size_t fixed_chunk)
{
  RunResult result;
  go2_video_bridge::H264StreamDecoder decoder(
    [&result](const AVFrame & frame) {
      ++result.frames;
      if (result.width == 0) {
        result.width = frame.width;
        result.height = frame.height;
      }
      const int packed_size = av_image_get_buffer_size(
        static_cast<AVPixelFormat>(frame.format), frame.width, frame.height, 1);
      if (frame.data[0] == nullptr || frame.width <= 0 || frame.height <= 0 || packed_size <= 0) {
        result.valid_frames = false;
      }
      if (frame.width != result.width || frame.height != result.height) {
        result.valid_frames = false;
      }
    });

  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const std::size_t chunk = std::min(fixed_chunk, bytes.size() - offset);
    decoder.push(bytes.data() + offset, chunk);
    offset += chunk;
  }
  decoder.flush();

  result.packets = static_cast<std::size_t>(decoder.packet_count());
  result.errors = static_cast<std::size_t>(decoder.error_count());
  return result;
}

RunResult run_with_random_chunks(const std::vector<std::uint8_t> & bytes)
{
  RunResult result;
  go2_video_bridge::H264StreamDecoder decoder(
    [&result](const AVFrame & frame) {
      ++result.frames;
      if (result.width == 0) {
        result.width = frame.width;
        result.height = frame.height;
      }
      const int packed_size = av_image_get_buffer_size(
        static_cast<AVPixelFormat>(frame.format), frame.width, frame.height, 1);
      if (frame.data[0] == nullptr || frame.width <= 0 || frame.height <= 0 || packed_size <= 0) {
        result.valid_frames = false;
      }
      if (frame.width != result.width || frame.height != result.height) {
        result.valid_frames = false;
      }
    });

  std::mt19937 generator(20260927U);
  std::uniform_int_distribution<std::size_t> distribution(1, 4096);
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const std::size_t chunk = std::min(distribution(generator), bytes.size() - offset);
    decoder.push(bytes.data() + offset, chunk);
    offset += chunk;
  }
  decoder.flush();

  result.packets = static_cast<std::size_t>(decoder.packet_count());
  result.errors = static_cast<std::size_t>(decoder.error_count());
  return result;
}

bool acceptable(const RunResult & result, const RunResult & baseline)
{
  return result.frames > 0 && result.packets > 0 && result.errors == 0 &&
         result.valid_frames && result.width > 0 && result.height > 0 &&
         result.frames == baseline.frames && result.width == baseline.width &&
         result.height == baseline.height;
}

void print_result(const std::string & label, const RunResult & result)
{
  std::cout << label << ": frames=" << result.frames
            << " packets=" << result.packets
            << " errors=" << result.errors
            << " resolution=" << result.width << "x" << result.height
            << " valid_frames=" << (result.valid_frames ? "true" : "false")
            << '\n';
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: h264_chunk_test ANNEX_B_H264_FILE\n";
    return 2;
  }

  std::ifstream input(argv[1], std::ios::binary);
  if (!input) {
    std::cerr << "cannot open " << argv[1] << '\n';
    return 2;
  }
  const std::vector<std::uint8_t> bytes(
    std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{});
  if (bytes.empty()) {
    std::cerr << "input is empty\n";
    return 2;
  }

  const std::vector<std::size_t> chunk_sizes{1, 7, 188, 1024, 4096};
  const RunResult baseline = run_with_chunks(bytes, chunk_sizes.front());
  print_result("chunk=1", baseline);

  bool all_passed = acceptable(baseline, baseline);
  for (std::size_t i = 1; i < chunk_sizes.size(); ++i) {
    const RunResult result = run_with_chunks(bytes, chunk_sizes[i]);
    print_result("chunk=" + std::to_string(chunk_sizes[i]), result);
    all_passed = acceptable(result, baseline) && all_passed;
  }

  const RunResult random_result = run_with_random_chunks(bytes);
  print_result("chunk=random(seed=20260927)", random_result);
  all_passed = acceptable(random_result, baseline) && all_passed;

  if (!all_passed) {
    std::cerr << "streaming chunk-boundary test failed\n";
    return 1;
  }
  std::cout << "streaming chunk-boundary test passed\n";
  return 0;
}
