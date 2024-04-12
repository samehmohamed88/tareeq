#include "platform/io/BoostSerialDeviceManager.hpp"
#include "platform/sensors/SensorRegistry.hpp"
#include "platform/vehicles/wave_rover/WaveRoverImuReader.hpp"

#include <fmt/format.h>

#include <iostream>
#include <memory>
#include <utility>

using namespace platform::io;


auto MakeWaveRoverImuRead(std::shared_ptr<BoostSerialDeviceManager> boostSerialDeviceManager, const std::string& port)
{
    return platform::vehicles::waverover::WaveRoverImuReader(std::move(boostSerialDeviceManager), port);
}

int main()
{
    auto port = std::string("/dev/ttyUSB0");
    auto boostSerialDeviceManager = std::make_shared<BoostSerialDeviceManager>();

    auto sensorRegistry = platform::sensors::SensorRegistry(MakeWaveRoverImuRead(boostSerialDeviceManager, port));
    sensorRegistry.getImuData();

    //    std::string request = getRetrieveImuCommand();
    //
    //    BoostSerialDeviceManager boostSerialDeviceManager_;
    //    auto status = boostSerialDeviceManager_.createDevice(port, 115200);
    //
    //    if (status.isSuccess()) {
    //        auto [status1, response] = boostSerialDeviceManager_.readFromDevice(port, request);
    //        if (response.has_value()) {
    //            std::cout << response.value() << std::endl;
    //        }
    //    }
    return 0;
}
