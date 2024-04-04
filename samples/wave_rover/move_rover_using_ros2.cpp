
#include "platform/ros2/actuators/MobileBaseActuator.hpp"

#include "platform/io/AsioOperationsImpl.hpp"
#include "platform/io/BoostNetworkIO.hpp"
#include "platform/logging/LoggerFactory.hpp"
#include "platform/vehicle/wave_rover/WaveRoverMotorController.hpp"
#include "platform/vehicle/wave_rover/WaveRoverIMUController.hpp"
#include "platform/vehicle/wave_rover/WaveRoverNetworkDeviceManager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <filesystem>
#include <cstdlib>

namespace fs = std::filesystem;
namespace rc = rclcpp;
namespace io = platform::io;
namespace wr = platform::vehicle::waverover;
namespace mt = platform::motors;
namespace lg = platform::logging;

using namespace rclcpp;
using namespace std::chrono_literals;

using namespace platform::io;
using namespace platform::vehicle::waverover;
using namespace platform::motors;
using namespace platform::logging;

using BoostNetworkDeviceType = BoostNetworkIO<AsioOperationsImpl, ConsoleLogger>;
using DeviceManagerType = WaveRoverNetworkDeviceManager<BoostNetworkDeviceType, ConsoleLogger>;
using MotorControllerType = WaveRoverMotorController<DeviceManagerType, ConsoleLogger>;

class WaveRoverManager
{
public:

    std::shared_ptr<AsioOperationsImpl> asioOperations_;
    std::shared_ptr<ConsoleLogger> logger_;
    std::shared_ptr<BoostNetworkDeviceType> boostNetworkIo_;
    std::shared_ptr<DeviceManagerType> deviceManager_;
    std::unique_ptr<MotorControllerType> motorController;

    std::shared_ptr<IMUPublisher<IMUController>> imu_publisher_;

    fs::path temp_log_dir;

    WaveRoverManager() {

        asioOperations_ = std::make_shared<AsioOperationsImpl>();
        logger_ = LoggerFactory::createLogger("console");

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
