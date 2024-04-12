
#include "platform/io/AsioOperationsImpl.hpp"
#include "platform/io/devices/BoostNetworkIO.hpp"
#include "platform/logging/LoggerFactory.hpp"
#include "platform/sensors/imu/IMUData.hpp"
#include "platform/vehicle/wave_rover/WaveRoverIMUController.hpp"
#include "platform/vehicle/wave_rover/WaveRoverNetworkDeviceManager.hpp"

#include <chrono>
#include <memory>
#include <thread>

int main()
{
    using namespace platform::io;
    using namespace platform::vehicle::waverover;
    using namespace platform::devices;
    using namespace platform::sensors::imu;

    using BoostNetworkDeviceType = BoostNetworkIO<AsioOperationsImpl, platform::logging::ConsoleLogger>;
    using DeviceManagerType = WaveRoverNetworkDeviceManager<BoostNetworkDeviceType, platform::logging::ConsoleLogger>;

    auto asioOperations = std::make_shared<AsioOperationsImpl>();

    auto logger = platform::logging::LoggerFactory::createLogger("console");

    auto boostNetworkIo = std::make_shared<BoostNetworkDeviceType>(asioOperations, logger);
    boostNetworkIo->initialize();

    auto deviceManager = std::make_shared<DeviceManagerType>(boostNetworkIo, logger);

    auto waveRoverIMUController = std::make_shared<WaveRoverIMUController<DeviceManagerType, platform::logging::ConsoleLogger>>(deviceManager, logger);

    //    waveRoverMotorController->setMotorPwm(50, 50);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    //    waveRoverMotorController->setMotorPwm(100, 100);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    //    waveRoverMotorController->setMotorPwm(150, 150);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto [error, data] = waveRoverIMUController->readData();
    if (data.has_value()) {
        // Access the IMUData if it exists
        std::cout << "Temperature: " << data->temp << std::endl;
        std::cout << "Roll: " << data->roll << std::endl;
    }
    return 0;
}
