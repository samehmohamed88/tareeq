
#include "platform/vehicle/wave_rover/WaveRoverROS2Manager.hpp"

int main() {
    using namespace platform::vehicle::waverover;
    auto rover = WaveRoverROS2Manager();
    rover.run();
    return 0;
}
