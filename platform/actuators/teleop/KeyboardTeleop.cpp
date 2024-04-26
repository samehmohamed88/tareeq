#include "platform/ros2/actuators/teleop/KeyboardTeleop.hpp"

#include <atomic>

namespace platform::ros2::teleop {

std::atomic<bool> KeyboardTeleop::should_exit_{false};  // Definition

// Definitions of constructor, destructor, and member functions

} // namespace platform::ros2::teleop