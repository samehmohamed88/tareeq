
#include "platform/ros2/localization/IMUPublisher.hpp"

#include "platform/io/AsioOperationsImpl.hpp"
#include "platform/io/BoostNetworkIO.hpp"
#include "platform/logging/LoggerFactory.hpp"
#include "platform/wave_rover/WaveRoverIMUController.hpp"
#include "platform/wave_rover/WaveRoverNetworkDeviceManager.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>

namespace fs = std::filesystem;

using namespace rclcpp;
using namespace std::chrono_literals;

using namespace platform::ros2::localization;
using namespace platform::waverover;

using namespace platform::io;
using namespace platform::waverover;
using namespace platform::devices;
using namespace platform::sensors::imu;

using BoostNetworkDeviceType = BoostNetworkIO<AsioOperationsImpl, ConsoleLogger>;
using DeviceManagerType = WaveRoverNetworkDeviceManager<BoostNetworkDeviceType, ConsoleLogger>;
using IMUController = WaveRoverIMUController<DeviceManagerType, ConsoleLogger>;

class IMUPublisherTest : public ::testing::Test
{
protected:

    std::shared_ptr<AsioOperationsImpl> asioOperations_;
    std::shared_ptr<ConsoleLogger> logger_;
    std::shared_ptr<BoostNetworkDeviceType> boostNetworkIo_;
    std::shared_ptr<DeviceManagerType> deviceManager_;
    std::unique_ptr<IMUController> waveRoverImuController_;

    std::shared_ptr<IMUPublisher<IMUController>> imu_publisher_;

    fs::path temp_log_dir;

//    Node::SharedPtr node;
//    rclcpp::NodeOptions options;

    void SetUp() override
    {

        asioOperations_ = std::make_shared<AsioOperationsImpl>();
        logger_ = LoggerFactory::createLogger("console");

        boostNetworkIo_ = std::make_shared<BoostNetworkDeviceType>(asioOperations_, logger_);
        boostNetworkIo_->initialize();

        deviceManager_ = std::make_shared<DeviceManagerType>(boostNetworkIo_, logger_);
        //        waveRoverImuController_ = ;

        temp_log_dir = fs::temp_directory_path() / "ros2_test_logs";
        fs::create_directories(temp_log_dir);           // Ensure the directory exists
        setenv("ROS_LOG_DIR", temp_log_dir.c_str(), 1); // Set the environment variable
        rclcpp::init(0, nullptr);


    }

    void TearDown() override
    {
        rclcpp::shutdown();
        fs::remove_all(temp_log_dir); // Clean up the temporary log directory
    }
};

TEST_F(IMUPublisherTest, ConstructIMUPublisher)
{
    rclcpp::NodeOptions options;
    // Configure node options with parameters
    options.allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);

    // Mock or set parameters as required by the node
    options.parameter_overrides({
        {"topic_name", "imu_data"},
        {"imu_frame_id", "imu_link"},
        {"publish_rate_ms", 20}
    });

    imu_publisher_ = std::make_shared<IMUPublisher<IMUController>>(
        std::make_unique<IMUController>(deviceManager_, logger_), options);

    // Limit the test duration
    auto start = std::chrono::steady_clock::now();
    auto timeout = 5s; // 5 seconds timeout for the test
    while (std::chrono::steady_clock::now() - start < timeout) {
        rclcpp::spin_some(imu_publisher_);
    }

    ASSERT_NE(imu_publisher_, nullptr);
}

TEST_F(IMUPublisherTest, ConstructIMUPublisherWithoutParameters)
{
    rclcpp::NodeOptions options;
    // Configure node options with parameters
    options.allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);

    // Mock or set parameters as required by the node
//    options.parameter_overrides({
//        {"topic_name", "imu_data"},
//        {"imu_frame_id", "imu_link"},
//        {"publish_rate_ms", 20}
//    });

    imu_publisher_ = std::make_shared<IMUPublisher<IMUController>>(
        std::make_unique<IMUController>(deviceManager_, logger_), options);

    // Limit the test duration
    auto start = std::chrono::steady_clock::now();
    auto timeout = 5s; // 5 seconds timeout for the test
    while (std::chrono::steady_clock::now() - start < timeout) {
        rclcpp::spin_some(imu_publisher_);
    }

    ASSERT_NE(imu_publisher_, nullptr);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    // Limit the test duration
    auto start = std::chrono::steady_clock::now();
    auto timeout = 5s; // 5 seconds timeout for the test
    while (std::chrono::steady_clock::now() - start < timeout) {
//        rclcpp::spin_some(imuPublisherTest.imu_publisher_);
    }
    return RUN_ALL_TESTS();
}
