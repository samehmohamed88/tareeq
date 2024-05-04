#include "platform/io/BoostSerialDeviceManager.hpp"

#include "platform/io/BoostSerialDevice.hpp"
#include "platform/io/Status.hpp"

#include <mutex>
#include <string>
#include <tuple>

namespace platform::io {

Status BoostSerialDeviceManager::initializeDevice(const std::string& port)
{
    std::lock_guard<std::mutex> lock(deviceMutex_);
    if (deviceMap_.contains(port) && deviceMap_[port]->isInitialized()) {
        logger_.logDebug("Initializing device.");
        return deviceMap_[port]->initialize();
    }
    logger_.logInfo("Device already initialized or not created.");
    return {STATUS::SUCCESS}; // Already initialized or device not created
}

Status BoostSerialDeviceManager::openDevice(const std::string& port)
{
    std::lock_guard<std::mutex> lock(deviceMutex_);
    if (deviceMap_.contains(port) && deviceMap_[port]->isOpened()) {
        logger_.logDebug("Opening device.");
        return deviceMap_[port]->open();
    }
    logger_.logWarn("Device already open or not created.");
    return {STATUS::ERROR, ERROR::NONE}; // Device already open or not created
}

Status BoostSerialDeviceManager::closeDevice(const std::string& port)
{
    std::lock_guard<std::mutex> lock(deviceMutex_);
    if (deviceMap_.contains(port) && deviceMap_[port]->isOpened()) {
        logger_.logDebug("Closing device.");
        return deviceMap_[port]->close();
    }
    logger_.logInfo("Device was not open or not created.");
    return {STATUS::SUCCESS}; // Device was not open or not created
}

std::tuple<Status, std::optional<std::string>> BoostSerialDeviceManager::readFromDevice(const std::string& port,
                                                                                        const std::string& request)
{
    std::lock_guard<std::mutex> lock(deviceMutex_);
    if (!deviceMap_.contains(port)) {
        logger_.logError("Device not created, cannot read.");
        return {Status(STATUS::ERROR, ERROR::COMMUNICATION_ERROR), {}};
    }
    logger_.logDebug("Reading from device: "+ request);
    return deviceMap_[port]->read(request);
}

Status BoostSerialDeviceManager::createDevice(const std::string& port, uint32_t baudRate)
{
    std::lock_guard<std::mutex> lock(deviceMutex_);
    logger_.logInfo("BoostSerialDeviceManager::createDevice ");
    if (!deviceMap_.contains(port)) {
        logger_.logInfo("Creating device at port: with baud rate: "+ port);
        try {
            deviceMap_[port] = std::make_unique<BoostSerialDevice>(port, baudRate);
            auto status = deviceMap_[port]->open();
            if (!status.isSuccess()) {
                logger_.logError("Failed to open newly created device.");
                return status;
            }
            logger_.logInfo("Device created and opened successfully.");
            return {STATUS::SUCCESS};
        } catch (const boost::system::system_error& e) {
            const boost::system::error_code& ec = e.code();
            if (ec == boost::asio::error::already_open) {
                logger_.logWarn("Device already open, attempting to close and reopen.");
                deviceMap_[port]->close();       // Try to close it first
                return deviceMap_[port]->open(); // Attempt to reopen
            } else if (ec == boost::asio::error::no_such_device || ec.value() == ENOENT) {
                std::string errorMessage = "Please make sure the device " + port + " is correct and accessible.";
                logger_.logError("No such device: {}", port);
                return {STATUS::ERROR, ERROR::ERROR_OPENING_DEVICE, errorMessage};
            } else {
                logger_.logError("Communication error while creating device: {}", e.what());
                return {STATUS::ERROR, ERROR::COMMUNICATION_ERROR, std::string(e.what())};
            }
        }
    }
    return {STATUS::SUCCESS}; // Device already created
}

Status BoostSerialDeviceManager::writeToDevice(const std::string& port, std::string&& data)
{
    std::lock_guard<std::mutex> lock(deviceMutex_);
    if (!deviceMap_.contains(port)) {
        logger_.logError("Device not created, cannot write data.");
        return {STATUS::ERROR, ERROR::SENSOR_FAILURE};
    }

    logger_.logInfo("Writing to device: "+ toHexString(data));
    return deviceMap_[port]->write(data);
}

} // namespace platform::io
