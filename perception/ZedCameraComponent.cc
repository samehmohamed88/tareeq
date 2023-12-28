#include "perception/ZedCameraComponent.h"

#include "component/Component.h"
#include "rclcpp/node_options.hpp"

namespace nav {
namespace perception {

ZedCameraComponent::ZedCameraComponent(const rclcpp::NodeOptions& options)
    : component::Component("", options)
{}

bool ZedCameraComponent::Init()
{
    return true;
}

void ZedCameraComponent::Run() {

}

} // namespace perception
} // namespace nav

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav::perception::ZedCameraComponent);


//#include "class_loader/register_macro.hpp"
//
//CLASS_LOADER_REGISTER_CLASS(MyCustomNodeFactory, rclcpp_components::NodeFactory)