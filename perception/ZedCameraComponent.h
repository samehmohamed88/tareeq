#pragma once

#include "component/Component.h"
#include "rclcpp/node_options.hpp"

namespace nav {
namespace perception {

class ZedCameraComponent : public component::Component
{
public:
    explicit ZedCameraComponent(const rclcpp::NodeOptions& options);

    bool Init() override;
    void Run() override;
};
} // namespace perception
} // namespace nav
