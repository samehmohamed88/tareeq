#include "platform/vehicle/wave_rover/WaveRoverNetworkDeviceManager.hpp"

#include <thread>
#include <chrono>

int main() {
    auto roverNetworkManager = platform::vehicle::waverover::WaveRoverNetworkDeviceManager();

    std::string target = R"(/js?json={"T":1,"L":88,"R":88})";
    for (int i = 0 ; i < 200; i ++) {
        roverNetworkManager.write(target);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
