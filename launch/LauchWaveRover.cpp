
#include "platform/sensors/zed/ZedCamera.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    auto zedCamera = std::make_shared<platform::zed::ZedCamera>();

    auto [status, data] = zedCamera->getImuData();
    std::cout << (*data).orientation.x << std::endl;
    std::cout << status.toString() << std::endl;
    rclcpp::init(argc, argv);

    rclcpp::shutdown();

    return 0;
}
