#pragma once

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_visual_slam_interfaces/srv/get_all_poses.hpp"

namespace platform::slam {

using std::placeholders::_1;
using GetAllPoses = isaac_ros_visual_slam_interfaces::srv::GetAllPoses;

class SLAMServiceClient : public rclcpp::Node {
public:
    SLAMServiceClient();
private:
    rclcpp::Client<GetAllPoses>::SharedPtr client_;
    GetAllPoses::Request::SharedPtr request_;

};

} // namespace platform::slam
