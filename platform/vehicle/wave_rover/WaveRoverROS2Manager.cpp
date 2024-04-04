#include "platform/vehicle/wave_rover/WaveRoverROS2Manager.hpp"

#include "platform/logging/LoggerFactory.hpp"

#include <memory>

namespace platform::vehicle::waverover {

WaveRoverROS2Manager::WaveRoverROS2Manager()
    : vehicleConfig_{0.0375, 0.13, .2}
    , asioOperations_{std::make_shared<io::AsioOperationsImpl>()}
    , logger_{logging::LoggerFactory::createLogger("console")}
    , boostNetworkIo_{std::make_shared<BoostNetworkDeviceType>(asioOperations_, logger_)}
    , deviceManager_{std::make_shared<DeviceManagerType>(boostNetworkIo_, logger_)}
    , motorController_{std::make_shared<MotorControllerType>(deviceManager_, logger_, vehicleConfig_)}
    , imuDeviceController_{std::make_shared<IMUControllerType>(deviceManager_, logger_)}
{


}

void WaveRoverROS2Manager::run()
{
    try {
        // Initialize ROS 2
        rclcpp::init(0, nullptr);

        // Create a multi-threaded executor
        rclcpp::executors::MultiThreadedExecutor executor;

        imuPublisher_ = makeIMUPublisher();
        mobileBaseActuator_ = makeMobileBaseActuator();

        // Add both nodes to the executor
        executor.add_node(imuPublisher_);
        executor.add_node(mobileBaseActuator_);

        // Spin both nodes
        executor.spin(); // This will block until the nodes are shutdown
    } catch (const rclcpp::exceptions::RCLInvalidArgument &e) {
        std::cerr << "RCLInvalidArgument exception caught: " << e.what() << std::endl;
        // Additional error handling
    } catch (const std::exception &e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        // Additional error handling
    }

}
} // namespace platform::vehicle::waverover
