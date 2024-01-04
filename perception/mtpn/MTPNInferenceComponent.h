#pragma once

#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>


namespace nav {
namespace perception {

class MTPNInferenceComponent : public rclcpp::Node {
public:
    MTPNInferenceComponent(const rclcpp::NodeOptions& options);
    bool Init();
    void Run();
private:
    void initSubscribers();
private:
    // ----> Messages (ONLY THOSE NOT CHANGING WHILE NODE RUNS)
    typedef std::unique_ptr<sensor_msgs::msg::Image> ImageMessage;
    typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> CameraInfoMessage;

    // ----> Subscribers
    image_transport::CameraSubscriber subscriberRgb_;
    image_transport::CameraSubscriber subscriberRawRgb_;
    image_transport::CameraSubscriber subscriberLeft_;
    image_transport::CameraSubscriber subscriberRawLeft_;
    image_transport::CameraSubscriber subscriberRight_;
    image_transport::CameraSubscriber subscriberRawRight_;
};

} // namespace perception
} // namespace nav
