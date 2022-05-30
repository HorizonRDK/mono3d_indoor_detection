// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

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
  std::string ai_msg_topic;
  node->get_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_topic);
  auto ai_sub = node->create_subscription<ai_msgs::msg::PerceptionTargets>(
          ai_msg_topic, 5, ai_msg_callback);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
