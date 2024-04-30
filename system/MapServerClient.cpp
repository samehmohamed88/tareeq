#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "isaac_ros_visual_slam_interfaces/action/save_map.hpp"

class MapSaverClient : public rclcpp::Node {
public:
    using SaveMap = isaac_ros_visual_slam_interfaces::action::SaveMap;
    using GoalHandleSaveMap = rclcpp_action::ClientGoalHandle<SaveMap>;

    explicit MapSaverClient(const std::string & name) : Node(name), map_url_("/workspace/maps/mymap") {
        this->client_ptr_ = rclcpp_action::create_client<SaveMap>(this, "visual_slam/save_map");
    }

    void send_goal() {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = SaveMap::Goal();
        goal_msg.map_url = map_url_;

        RCLCPP_INFO(this->get_logger(), "Sending save map request for map at '%s'", map_url_.c_str());

        auto send_goal_options = rclcpp_action::Client<SaveMap>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MapSaverClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&MapSaverClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&MapSaverClient::result_callback, this, std::placeholders::_1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<SaveMap>::SharedPtr client_ptr_;
    std::string map_url_;

    void goal_response_callback(GoalHandleSaveMap::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        }
    }

    void feedback_callback(
        GoalHandleSaveMap::SharedPtr,
        const std::shared_ptr<const SaveMap::Feedback> feedback) {
        // Process feedback if feedback is expected
    }

    void result_callback(const GoalHandleSaveMap::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Map saved successfully");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Save map aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Save map canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapSaverClient>("map_saver_client");
    node->send_goal();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
