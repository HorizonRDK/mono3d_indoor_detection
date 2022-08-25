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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#ifdef CV_BRIDGE_PKG_ENABLED
#include "cv_bridge/cv_bridge.h"
#endif
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "include/centernet_3d_output_parser.h"
#include "include/image_utils.h"
#include "sensor_msgs/msg/image.hpp"
#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#ifndef INCLUDE_CENTERNET3D_DETECTION_NODE_H_
#define INCLUDE_CENTERNET3D_DETECTION_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::TaskId;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::NV12PyramidInput;

struct CenterNet3DOutput : public DnnNodeOutput {
  std::shared_ptr<std_msgs::msg::Header> image_msg_header_ = nullptr;
  uint32_t src_img_width_;
  uint32_t src_img_height_;
  std::string image_name_;
  std::shared_ptr<cv::Mat> mat_ = nullptr;
};

class CenterNet3DDetectionNode : public DnnNode {
 public:
  CenterNet3DDetectionNode(const std::string &node_name,
                    const NodeOptions &options = NodeOptions());
  ~CenterNet3DDetectionNode() override;

  int PredictByImage(const std::string &image_name);

 protected:
  int SetNodePara() override;
  int SetOutputParser() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  void GetConfig();
  int Start();
  int Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
              const std::shared_ptr<std::vector<hbDNNRoi>> rois,
              std::shared_ptr<DnnNodeOutput> dnn_output);
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
#ifdef SHARED_MEM_ENABLED
  void SharedMemImgProcess(
      const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif

 private:
  // 输入参数
  std::string config_file_path_ = "./config";
  int shared_mem_ = 0;
  std::string feed_image_ = "";// "./config/images/3d_detection.png";

  std::string model_file_name_ = "config/centernet.hbm";
  std::string model_name_ = "centernet";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;
  int model_input_width_ = 960;
  int model_input_height_ = 512;
  const int32_t model_output_count_ = 6;
  const int32_t output_index_ = 5;
  float score_threshold_ = 0.5;

  std::chrono::high_resolution_clock::time_point output_tp_;
  std::atomic_int output_frameCount_;
  std::mutex frame_stat_mtx_;
  std::string msg_pub_topic_name_ = "ai_msg_3d_detection";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_;

#ifdef SHARED_MEM_ENABLED
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
#endif
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  // 目前只支持订阅原图，可以使用压缩图"/image_raw/compressed" topic
  // 和sensor_msgs::msg::CompressedImage格式扩展订阅压缩图
  std::string ros_img_topic_name_ = "/image_raw";

  int dump_render_img_ = 0;
};

#endif  // INCLUDE_CENTERNET3D_DETECTION_NODE_H_
