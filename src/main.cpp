// Copyright (c) 2022，Horizon Robotics.
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

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "include/centernet_3d_detection_node.h"
#include "ai_msgs/msg/perception_targets.hpp"

void ai_msg_callback(ai_msgs::msg::PerceptionTargets::SharedPtr ai_msg) {
  const auto &targets = ai_msg->targets;
  static auto logger = rclcpp::get_logger("ai_msg_cb");
  for (const auto &target : targets) {
    RCLCPP_INFO_STREAM(logger, "target type: " << target.type);
    for (const auto &attr : target.attributes) {
      RCLCPP_INFO_STREAM(logger, "attribute type: " << attr.type
                                  << ", value: " << attr.value / 1000.);
    }
    for (const auto &point : target.points) {
      RCLCPP_INFO_STREAM(logger,"point type: " << point.type);
      for (const auto &p : point.point) {
        RCLCPP_INFO_STREAM(logger, "x: " << p.x << " y: " << p.y);
      }
    }
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CenterNet3DDetectionNode>("mono3d_dection_node");
  // std::string ai_msg_topic;
  // node->get_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_topic);
  // auto ai_sub = node->create_subscription<ai_msgs::msg::PerceptionTargets>(
  //         ai_msg_topic, 5, ai_msg_callback);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
