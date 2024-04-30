#include "platform/slam/SLAMActionClient.hpp"

#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

namespace platform::slam {

SLAMActionClient::SLAMActionClient(const rclcpp::NodeOptions& node_options)
    : Node("minimal_action_client", node_options)
    , goal_done_(false)
{
    this->client_ptr_ = rclcpp_action::create_client<SaveMap>(this->get_node_base_interface(),
                                                              this->get_node_graph_interface(),
                                                              this->get_node_logging_interface(),
                                                              this->get_node_waitables_interface(),
                                                              "save_slam_map");

    this->timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&SLAMActionClient::sendGoal, this));
}

void SLAMActionClient::sendGoal()
{
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        this->goal_done_ = true;
        return;
    }

    auto goal_msg = SaveMap::Goal();
    goal_msg.map_url = getCurrentDateTime();

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<SaveMap>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](auto&& PH1) {
        goalResponseCallback(std::forward<decltype(PH1)>(PH1));
    };
    send_goal_options.feedback_callback = [this](auto&& PH1, auto&& PH2) {
        feedbackCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    send_goal_options.result_callback = [this](auto&& PH1) { resultCallback(std::forward<decltype(PH1)>(PH1)); };
    //        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void SLAMActionClient::resultCallback(const GoalHandleSaveMap::WrappedResult& result)
{
    this->goal_done_ = true;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }

    RCLCPP_INFO(this->get_logger(), "Success received.  Map Saved");
}

void SLAMActionClient::feedbackCallback(GoalHandleSaveMap::SharedPtr,
                                        const std::shared_ptr<const SaveMap::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Next number in sequence received: %d", feedback->progress);
}

void SLAMActionClient::goalResponseCallback(GoalHandleSaveMap::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

} // namespace platform::slam
