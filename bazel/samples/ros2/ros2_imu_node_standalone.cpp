
#include "platform/io/AsioOperationsImpl.hpp"
#include "platform/io/devices/BoostNetworkIO.hpp"
#include "platform/logging/LoggerFactory.hpp"
#include "platform/ros2/localization/IMUPublisher.hpp"
#include "platform/vehicle/wave_rover/WaveRoverIMUController.hpp"
#include "platform/vehicle/wave_rover/WaveRoverNetworkDeviceManager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>

namespace fs = std::filesystem;

using namespace rclcpp;
using namespace std::chrono_literals;

using namespace platform::ros2::localization;
using namespace platform::vehicle::waverover;

using namespace platform::io;
using namespace platform::vehicle::waverover;
using namespace platform::devices;
using namespace platform::sensors::imu;

using BoostNetworkDeviceType = BoostNetworkIO<AsioOperationsImpl, platform::logging::ConsoleLogger>;
using DeviceManagerType = WaveRoverNetworkDeviceManager<BoostNetworkDeviceType, platform::logging::ConsoleLogger>;
using IMUController = WaveRoverIMUController<DeviceManagerType, platform::logging::ConsoleLogger>;

class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        subscription_ = create_subscription<sensor_msgs::msg::Imu>(
            "topic", 10, [this](const sensor_msgs::msg::Imu& msg) {
                RCLCPP_INFO(get_logger(), "I heard: '%f'", msg.orientation.x);
            });
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

class IMUPublisherTest
{
public:

    std::shared_ptr<AsioOperationsImpl> asioOperations_;
    std::shared_ptr<platform::logging::ConsoleLogger> logger_;
    std::shared_ptr<BoostNetworkDeviceType> boostNetworkIo_;
    std::shared_ptr<DeviceManagerType> deviceManager_;
    std::unique_ptr<IMUController> waveRoverImuController_;

    std::shared_ptr<IMUPublisher<IMUController>> imu_publisher_;

    fs::path temp_log_dir;

    IMUPublisherTest() {

        asioOperations_ = std::make_shared<AsioOperationsImpl>();
        logger_ = platform::logging::LoggerFactory::createLogger("console");

        boostNetworkIo_ = std::make_shared<BoostNetworkDeviceType>(asioOperations_, logger_);
        boostNetworkIo_->initialize();

        deviceManager_ = std::make_shared<DeviceManagerType>(boostNetworkIo_, logger_);

    }
};


int main(int argc, char* argv[]) {

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    IMUPublisherTest imuPublisherTest{};

    rclcpp::NodeOptions options;
    // Configure node options with parameters
    options.allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);

    // Mock or set parameters as required by the node
    options.parameter_overrides({
        {"topic_name", "imu_data"},
        {"imu_frame_id", "imu_link"},
        {"publish_rate_ms", 10}
    });

    imuPublisherTest.imu_publisher_ = std::make_shared<IMUPublisher<IMUController>>(
        std::make_unique<IMUController>(imuPublisherTest.deviceManager_, imuPublisherTest.logger_), options);

    // Create a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add both nodes to the executor
    executor.add_node(imuPublisherTest.imu_publisher_);
    executor.add_node(std::make_shared<MinimalSubscriber>());

    // Spin both nodes
    executor.spin();  // This will block until the nodes are shutdown

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
