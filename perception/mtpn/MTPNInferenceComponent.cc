#include "perception/mtpn/MTPNInferenceComponent.h"

#include <rclcpp/node_options.hpp>

namespace nav {
namespace perception {

MTPNInferenceComponent::MTPNInferenceComponent(const rclcpp::NodeOptions& options)
: rclcpp::Node("ZedCameraComponent", options)
{}

void MTPNInferenceComponent::initSubscribers() {

}

} // namespace perception
} // namespace nav
