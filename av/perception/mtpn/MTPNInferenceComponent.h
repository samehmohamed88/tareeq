#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace av {
namespace perception {

class MTPNInferenceComponent : public rclcpp::Node
{
public:
    MTPNInferenceComponent(const rclcpp::NodeOptions& options);
    bool Init();
    void Run();

private:
    void initSubscribers();
    void subscriptionCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                              const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info);

private:
    // ----> Messages (ONLY THOSE NOT CHANGING WHILE NODE RUNS)
    typedef std::unique_ptr<sensor_msgs::msg::Image> ImageMessage;
    typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> CameraInfoMessage;

    // ----> Subscribers
    std::string topicRoot_ = "~/";
    image_transport::CameraSubscriber leftImageSubscriber_;
    //    image_transport::CameraSubscriber subscriberRawLeft_;
    //    image_transport::CameraSubscriber subscriberRight_;
    //    image_transport::CameraSubscriber subscriberRawRight_;
};

} // namespace perception
} // namespace nav
