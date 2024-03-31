
#include "platform/devices/BoostSerialPort.hpp"
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

    using BoostSerialDeviceType = BoostSerialPort<AsioOperationsImpl, ConsoleLogger>;
    using SerialDeviceManagerType = SerialDeviceManager<BoostSerialDeviceType, ConsoleLogger>;

    auto asioOperations = std::make_shared<AsioOperationsImpl>();

    auto logger = LoggerFactory::createLogger("console");

    auto boostSerialDevice = std::make_shared<BoostSerialDeviceType>(asioOperations, logger, "/dev/ttyTHS0");
    boostSerialDevice->initialize();

    auto deviceManager = std::make_shared<SerialDeviceManagerType>(boostSerialDevice, logger);

    auto waveRoverMotorController = std::make_shared<WaveRoverMotorController<SerialDeviceManagerType, ConsoleLogger>>(deviceManager, logger);

    waveRoverMotorController->setWheelSpeeds(164, 164);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    waveRoverMotorController->stop();
    return 0;
}
