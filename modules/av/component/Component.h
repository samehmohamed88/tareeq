#pragma once

#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>

namespace nav {
namespace component {

class ComponentConfig
{
public:
    ComponentConfig(std::string nodeName)
        : nodeName_{std::move(nodeName)}
    {}

protected:
    const std::string nodeName_;
};

class TimerComponentConfig : public ComponentConfig
{
public:
    TimerComponentConfig(std::string nodeName, int timerValue)
        : ComponentConfig(std::move(nodeName))
        , timerValue_{timerValue}
    {}

    const int timerValue_;
};

class Component : public rclcpp::Node
{
public:
    explicit Component(std::string nodeName, const rclcpp::NodeOptions& options)
        : rclcpp::Node(nodeName, options)
        , nodeName_{std::move(nodeName)} {};

    virtual bool Init() = 0;
    virtual void Run() = 0;

protected:
    const std::string nodeName_;
    bool isInitialized_ = false;
};

//class TimerConfig
//{
//    int timeValue_;
//};

//class TimerComponent : public Component
//{
//public:
//    explicit TimerComponent(std::string nodeName, const rclcpp::NodeOptions& options)
//        : Component(std::move(nodeName), options)
////        , config_{config}
//    {}
////    virtual bool Init() = 0;
////    virtual void Run() = 0;
//private:
////    TimerConfig config_;
//};

} //  namespace component
} // namespace nav
