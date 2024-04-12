#include "platform/vehicle/wave_rover/WaveRoverROS2Manager.hpp"

#include "platform/logging/LoggerFactory.hpp"

#include <cstdlib> // For std::exit()
#include <memory>

namespace platform::vehicle::waverover {

WaveRoverROS2Manager::WaveRoverROS2Manager()
    : vehicleConfig_{0.0375, 0.13, .1}
    , logger_{logging::LoggerFactory::createLogger("console")}
    , deviceManager_{std::make_shared<DeviceManagerType>()}
    , motorController_{std::make_shared<MotorControllerType>(deviceManager_, logger_, vehicleConfig_)}
    , zedCamera_{std::make_shared<platform::zed::ZEDCamera>()}
    , imu1Controller_{std::make_shared<IMU1ControllerType>(zedCamera_)}
    , imu2Controller_{std::make_shared<IMU2ControllerType>()}
{}

void WaveRoverROS2Manager::run()
{
    try {
        // Initialize ROS 2
        rclcpp::init(0, nullptr);

        // Create a multi-threaded executor
//        rclcpp::executors::MultiThreadedExecutor executor;
        rclcpp::executors::SingleThreadedExecutor executor;

        imuPublisher_ = makeIMUPublisher();
        mobileBaseActuator_ = makeMobileBaseActuator();
        stateEstimationNode_ = std::make_shared<localization::StateEstimationNode>();

        // Add nodes to the executor
        executor.add_node(imuPublisher_);
        executor.add_node(mobileBaseActuator_);
        executor.add_node(stateEstimationNode_);

        executor.spin(); // This will block until the nodes are shutdown
        rclcpp::shutdown();
    } catch (const rclcpp::exceptions::RCLInvalidArgument& e) {
        std::cerr << "RCLInvalidArgument exception caught: " << e.what() << std::endl;
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        rclcpp::shutdown();
    }
}
} // namespace platform::vehicles::waverover
