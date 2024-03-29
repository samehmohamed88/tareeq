#pragma once



namespace av::devices {

template <typename SerialPortImpl>
class SerialDevice {
public:
    SerialDevice(SerialPortImpl& serialPort)
        : serialPort_(serialPort) {}

    void write(const std::string& data) {
        serialPort_.write(data);
    }

    std::string read() {
        return serialPort_.read();
    }

private:
    SerialPortImpl& serialPort_;
};

} // namespace av::devices
