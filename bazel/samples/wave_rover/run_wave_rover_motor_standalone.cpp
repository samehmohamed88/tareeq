
#include "platform/devices/DeviceManager.hpp"
#include "platform/io/AsioOperationsImpl.hpp"
#include "platform/io/devices/BoostNetworkIO.hpp"
#include "platform/logging/LoggerFactory.hpp"
#include "platform/vehicle/wave_rover/WaveRoverMotorController.hpp"

#include <chrono>
#include <memory>
#include <thread>

int main()
{
    using namespace platform::io;
    using namespace platform::vehicle::waverover;
    using namespace platform::devices;
    using namespace platform::motors;

    using BoostNetworkDeviceType = BoostNetworkIO<AsioOperationsImpl, ConsoleLogger>;
    using DeviceManagerType = DeviceManager<BoostNetworkDeviceType, ConsoleLogger>;

    auto asioOperations = std::make_shared<AsioOperationsImpl>();

    auto logger = LoggerFactory::createLogger("console");

    auto boostSerialDevice = std::make_shared<BoostNetworkDeviceType>(asioOperations, logger);
    boostSerialDevice->initialize();

    auto deviceManager = std::make_shared<DeviceManagerType>(boostSerialDevice, logger);

    auto waveRoverMotorController = std::make_shared<WaveRoverMotorController<DeviceManagerType, ConsoleLogger>>(deviceManager, logger);

    waveRoverMotorController->setMotorPwm(50, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//    waveRoverMotorController->setMotorPwm(100, 100);
//    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//    waveRoverMotorController->setMotorPwm(150, 150);
//    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    waveRoverMotorController->stop();
    return 0;
}
