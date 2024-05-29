#include "platform/slam/SLAMServiceClient.hpp"

#include <memory>

namespace platform::slam {

using std::placeholders::_1;
using GetAllPoses = isaac_ros_visual_slam_interfaces::srv::GetAllPoses;

SLAMServiceClient::SLAMServiceClient()
    : rclcpp::Node("SLAMServiceClient")
    , client_{this->create_client<GetAllPoses>("get_slam_poses")}
    , request_{std::make_shared<GetAllPoses::Request>()}
{
    request_->max_count = 1;
}

} // namespace platform::slam
