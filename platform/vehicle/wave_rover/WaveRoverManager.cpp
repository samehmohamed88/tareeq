#include "platform/vehicle/wave_rover/WaveRoverManager.hpp"

namespace platform::vehicle::waverover {
WaveRoverManager::WaveRoverManager()
    : asioOperations_{std::make_shared<io::AsioOperationsImpl>()}
    ,
    {}
}