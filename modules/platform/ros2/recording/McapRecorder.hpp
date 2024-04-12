#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcpputils/env.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include "rosbag2_cpp/writer.hpp"

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <mutex>
#include <iostream>

namespace platform::ros2::recording {

class McapRecorder
{
public:
    McapRecorder(const std::string& mcapFilePath = "/home/sameh/tareeq", const std::string& storageId = "mcap")
    {
        std::string currentTimeAsString = getCurrentDateTimeStamp();
        auto uri = rcpputils::fs::path(mcapFilePath) / currentTimeAsString;

        rosbag2_storage::StorageOptions storageOptions;
        storageOptions.uri = uri.string();
        storageOptions.storage_id = storageId;

        // Prepare the rosbag2 objects for recording
        // A sequential writer is used as this is the only writer currently available
        writer_ = std::make_shared<rosbag2_cpp::Writer>(
            std::make_unique<rosbag2_cpp::writers::SequentialWriter>());

        // Open the mcap file
      writer_->open(storageOptions);
    }

    template<class M>
    void recordMessage(const std::string& topicName, const M& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (topicCreated(topicName)) {

            rclcpp::Serialization<M> serialization;
            auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
            serialization.serialize_message(&msg, serialized_msg.get());

            auto serialized_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            serialized_bag_msg->topic_name = topicName;

            serialized_bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                const_cast<rcutils_uint8_array_t*>(&serialized_msg->get_rcl_serialized_message()),
                [](rcutils_uint8_array_t* /* data */) {});

            // TODO: potentially grab the nano seconds since epoch from the header but that requires
            // additional parsing, and some messages dont have headers
            // Convert the time point to nanoseconds since the epoch
            auto now = std::chrono::system_clock::now();
            auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

            serialized_bag_msg->time_stamp = nanoseconds;
            writer_->write(serialized_bag_msg);
        }
    }

    void createTopic(const std::string& topicName,
                     const std::string& type,
                     const std::string& serializationFormat = "cdr")
    {
        if (!topicCreated(topicName)) {
            rosbag2_storage::TopicMetadata topicMetadata;
            topicMetadata.name = topicName;
            topicMetadata.type = type;
            topicMetadata.serialization_format = serializationFormat;

            writer_->create_topic(topicMetadata);

            topicNames_.insert(topicName);
        }
    }

private:
    bool topicCreated(const std::string& item) {
        return topicNames_.find(item) != topicNames_.end();
    }

    std::string getCurrentDateTimeStamp()
    {
        // Get the current time point
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        // Convert to tm struct for formatting
        std::tm bt = *std::localtime(&in_time_t);

        // Format the datetime string
        std::ostringstream oss;
        oss << std::put_time(&bt, "%Y-%m-%d-%H-%M-%S");
        return oss.str();
    }

    // Utility to check if a type has a 'header.stamp' member
    template<typename T, typename = void>
    struct has_header_stamp : std::false_type {};

    template<typename T>
    struct has_header_stamp<T, std::void_t<decltype(std::declval<T>().header.stamp)>> : std::true_type {};

    std::shared_ptr<rosbag2_cpp::Writer> writer_;
    std::unordered_set<std::string> topicNames_;
    std::mutex mutex_;
};

} // namespace platform::ros2::recording
