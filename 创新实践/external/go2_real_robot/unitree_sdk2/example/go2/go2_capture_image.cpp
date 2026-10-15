#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

namespace
{
constexpr std::size_t kMaxJpegBytes = 10U * 1024U * 1024U;

bool IsJpeg(const std::vector<uint8_t>& image)
{
    return image.size() >= 4U && image[0] == 0xFFU && image[1] == 0xD8U &&
           image[image.size() - 2U] == 0xFFU && image[image.size() - 1U] == 0xD9U;
}
}

int main(int argc, char** argv)
{
    const std::string network_interface = argc >= 2 ? argv[1] : "eth0";
    const std::string output_path = argc >= 3 ? argv[2] : "go2_front.jpg";

    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    unitree::robot::go2::VideoClient video_client;
    video_client.SetTimeout(3.0f);
    video_client.Init();

    std::vector<uint8_t> image;
    const int32_t ret = video_client.GetImageSample(image);
    if (ret != 0) {
        std::cerr << "GetImageSample failed ret=" << ret << std::endl;
        return 1;
    }
    if (image.size() > kMaxJpegBytes || !IsJpeg(image)) {
        std::cerr << "invalid image payload bytes=" << image.size() << std::endl;
        return 2;
    }

    std::ofstream output(output_path, std::ios::binary);
    if (!output) {
        std::cerr << "cannot open output path: " << output_path << std::endl;
        return 3;
    }
    output.write(reinterpret_cast<const char*>(image.data()), static_cast<std::streamsize>(image.size()));
    if (!output) {
        std::cerr << "failed to write output path: " << output_path << std::endl;
        return 4;
    }

    std::cout << "saved_jpeg=" << output_path << " bytes=" << image.size() << std::endl;
    return 0;
}
