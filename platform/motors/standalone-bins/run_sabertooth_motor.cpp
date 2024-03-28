
#include "av/motors/SabertoothMotorController.hpp"

using namespace av::motors;

int main() {
    SabertoothDriver driver("/dev/serial0", SabertoothDriver::DriveType::Differential);
    auto result = driver.initialize();
    if (std::holds_alternative<SabertoothDriver::Error>(result)) {
        std::cerr << "Initialization failed" << std::endl;
        return -1;
    }

    driver.setSpeed(30); // Set speed
    driver.steer(15);    // Steer with an angle or differential value

    return 0;
}
