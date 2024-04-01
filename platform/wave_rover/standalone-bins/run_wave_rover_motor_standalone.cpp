
#include "platform/devices/BoostNetworkDevice.hpp"
#include "platform/devices/SerialDeviceManager.hpp"
#include "platform/io/AsioOperationsImpl.hpp"
#include "platform/logging/LoggerFactory.hpp"
#include "platform/motors/WaveRoverMotorController.hpp"

#include <memory>
#include <thread>
#include <chrono>

int main()
{
    using namespace platform::io;
    using namespace platform::devices;
    using namespace platform::motors;

    using BoostNetworkDeviceType = BoostNetworkDevice<AsioOperationsImpl, ConsoleLogger>;
    using SerialDeviceManagerType = SerialDeviceManager<BoostNetworkDeviceType, ConsoleLogger>;

    auto asioOperations = std::make_shared<AsioOperationsImpl>();

    auto logger = LoggerFactory::createLogger("console");

    auto boostSerialDevice = std::make_shared<BoostNetworkDeviceType>(asioOperations, logger);
    boostSerialDevice->initialize();

    auto deviceManager = std::make_shared<SerialDeviceManagerType>(boostSerialDevice, logger);

    auto waveRoverMotorController = std::make_shared<WaveRoverMotorController<SerialDeviceManagerType, ConsoleLogger>>(deviceManager, logger);

    waveRoverMotorController->setMotorPwm(50, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    waveRoverMotorController->setMotorPwm(100, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    waveRoverMotorController->setMotorPwm(150, 150);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    waveRoverMotorController->stop();
    return 0;
}
