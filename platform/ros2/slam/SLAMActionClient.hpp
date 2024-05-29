#pragma once

#include <isaac_ros_visual_slam_interfaces/action/save_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <iostream>
#include <string>

namespace platform::slam {

class SLAMActionClient : public rclcpp::Node
{
public:
    using SaveMap = isaac_ros_visual_slam_interfaces::action::SaveMap;
    using GoalHandleSaveMap = rclcpp_action::ClientGoalHandle<SaveMap>;

private:
    rclcpp_action::Client<SaveMap>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;

    std::string getCurrentDateTime()
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
        return std::move("/workspace/maps/"+ss.str());
    }

public:
    explicit SLAMActionClient(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

    bool is_goal_done() const { return this->goal_done_; }

    void sendGoal();

    void goalResponseCallback(GoalHandleSaveMap::SharedPtr goal_handle);

    void feedbackCallback(GoalHandleSaveMap::SharedPtr, std::shared_ptr<const SaveMap::Feedback> feedback);

    void resultCallback(const GoalHandleSaveMap::WrappedResult& result);

}; // class MinimalActionClient

} // namespace platform::slam
