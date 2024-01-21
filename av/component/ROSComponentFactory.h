#pragma once

#include "class_loader/class_loader.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp_components/node_factory.hpp"

namespace nav {
namespace component {

class ROSComponentFactory : public rclcpp_components::NodeFactory
{
public:
    rclcpp_components::NodeInstanceWrapper create_node_instance(const rclcpp::NodeOptions& options) override;

    template<class ComponentConfig>
    rclcpp_components::NodeInstanceWrapper createComponent(ComponentConfig config,
                                                           const rclcpp::NodeOptions& options);
};

template<class ComponentConfig>
rclcpp_components::NodeInstanceWrapper ROSComponentFactory::createComponent(const ComponentConfig config,
                                                                            const rclcpp::NodeOptions& options)
{

}
} // namespace component
} // namespace nav
