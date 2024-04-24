#include "perception/mtpn/MTPNInferenceComponent.h"

#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/node_options.hpp>
#include <cv_bridge/cv_bridge.hpp>


#include <string>

namespace av {
namespace perception {

MTPNInferenceComponent::MTPNInferenceComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ZedCameraComponent", options)
{}

void MTPNInferenceComponent::initSubscribers()
{
    // ----> Topics names definition
    std::string rightTopicRoot = "right";
    std::string leftTopicRoot = "left";
    std::string imgTopic = "/image_rect_color";
    std::string imgRawTopic = "/image_raw_color";
    std::string rawSuffix = "_raw";
    std::string leftTopic = topicRoot_ + leftTopicRoot + imgTopic;
    std::string leftRawTopic = topicRoot_ + leftTopicRoot + rawSuffix + imgRawTopic;
    std::string rightTopic = topicRoot_ + rightTopicRoot + imgTopic;
    std::string rightRawTopic = topicRoot_ + rightTopicRoot + rawSuffix + imgRawTopic;
    // <---- Topics names definition
    leftImageSubscriber_ = image_transport::create_camera_subscription(
        this,
        leftTopic,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr& image,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfo) {
            this->subscriptionCallback(image, cameraInfo);
        },
        "raw");
}

void subscriptionCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfo)
{

}

} // namespace perception
} // namespace nav
