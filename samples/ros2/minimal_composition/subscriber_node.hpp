// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MINIMAL_COMPOSITION__SUBSCRIBER_NODE_HPP_
#define MINIMAL_COMPOSITION__SUBSCRIBER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "samples/ros2/minimal_composition/visibility.h"

class SubscriberNode : public rclcpp::Node
{
public:
  MINIMAL_COMPOSITION_PUBLIC SubscriberNode(rclcpp::NodeOptions options);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // MINIMAL_COMPOSITION__SUBSCRIBER_NODE_HPP_
