#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcpputils/env.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <rosbag2_storage/storage_factory.hpp>
//#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        auto uri = rcpputils::fs::path("/tareeq") / "bag";
        const int64_t timestamp_nanos = 100; // arbitrary value
        rcutils_time_point_value_t time_stamp{timestamp_nanos};
        const std::string topic_name = "test_topic";
        const std::string message_data = "Test Message 1";
        const std::string storage_id = "mcap";

        rclcpp::Serialization<std_msgs::msg::String> serialization;

        rosbag2_storage::TopicMetadata topic_metadata;
        topic_metadata.name = topic_name;
        topic_metadata.type = "std_msgs/msg/String";
        topic_metadata.serialization_format = "cdr";

        std_msgs::msg::String msg;
        msg.data = message_data;

        rosbag2_storage::StorageFactory factory;

        rosbag2_storage::StorageOptions options;
        options.uri = uri.string();
        options.storage_id = storage_id;

        auto writer = factory.open_read_write(options);

        writer->create_topic(topic_metadata);

        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
        serialization.serialize_message(&msg, serialized_msg.get());

        auto serialized_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        serialized_bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            const_cast<rcutils_uint8_array_t*>(&serialized_msg->get_rcl_serialized_message()),
            [](rcutils_uint8_array_t* /* data */) {});
        serialized_bag_msg->time_stamp = time_stamp;
        serialized_bag_msg->topic_name = topic_name;
        writer->write(serialized_bag_msg);


    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}