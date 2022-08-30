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

#include "include/centernet_3d_detection_node.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node.h"
#include "include/image_utils.h"
#include "rclcpp/rclcpp.hpp"
#include <sys/stat.h>
#ifdef CV_BRIDGE_PKG_ENABLED
#include <cv_bridge/cv_bridge.h>
#endif

using hobot::easy_dnn::OutputDescription;
using hobot::easy_dnn::OutputParser;

CenterNet3DDetectionNode::CenterNet3DDetectionNode(const std::string &node_name,
                                     const NodeOptions &options)
    : DnnNode(node_name, options), output_frameCount_(0) {
  this->declare_parameter<std::string>("config_file_path", config_file_path_);
  this->declare_parameter<int>("shared_mem", shared_mem_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name",
                                       msg_pub_topic_name_);
  this->declare_parameter<std::string>("image_sub_topic_name",
                                       ros_img_topic_name_);
  this->declare_parameter<std::string>("feed_image", feed_image_);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);

  this->get_parameter<std::string>("config_file_path", config_file_path_);
  this->get_parameter<int>("shared_mem", shared_mem_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name",
                                   msg_pub_topic_name_);
  this->get_parameter<std::string>("image_sub_topic_name", ros_img_topic_name_);
  this->get_parameter<std::string>("feed_image", feed_image_);
  this->get_parameter<int>("dump_render_img", dump_render_img_);

  model_file_name_ = config_file_path_ + "/centernet.hbm";

  mkdir("./result/", 666);

  std::stringstream ss;
  ss << "Parameter:"
     << "\n config_file_path_:" << config_file_path_
     << "\n model_file_name_: " << model_file_name_
     << "\n feed_image:" << feed_image_
     << "\n dump_render_img:" << dump_render_img_;
  RCLCPP_WARN(rclcpp::get_logger("mono3d_indoor_detection"), "%s",
              ss.str().c_str());
  if (Start() == 0) {
    RCLCPP_WARN(rclcpp::get_logger("mono3d_indoor_detection"),
                "start success!!!");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "start fail!!!");
  }
}

CenterNet3DDetectionNode::~CenterNet3DDetectionNode() {}

int CenterNet3DDetectionNode::Start() {
  int ret = Init();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"), "Init failed!");
    return ret;
  }

  ret = GetModelInputSize(0, model_input_width_, model_input_height_);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "Get model input size fail!");
    return ret;
  }
  RCLCPP_INFO(rclcpp::get_logger("mono3d_indoor_detection"),
              "The model input width is %d and height is %d",
              model_input_width_, model_input_height_);
  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      msg_pub_topic_name_, 10);
  RCLCPP_INFO(rclcpp::get_logger("mono3d_indoor_detection"),
              "msg_pub_topic_name: %s", msg_pub_topic_name_.data());
  if (!feed_image_.empty()) {
    std::cout << "mono3d read image:" << feed_image_
              << " to detect" << std::endl;
    PredictByImage(feed_image_);
    return 0;
  }

  //
  RCLCPP_INFO(rclcpp::get_logger("mono3d_indoor_detection"),
              "Detect images that use subscriptions");

  if (shared_mem_) {
#ifdef SHARED_MEM_ENABLED
    RCLCPP_WARN(rclcpp::get_logger("mono3d_indoor_detection"),
                "Create hbmem_subscription with topic_name: %s",
                sharedmem_img_topic_name_.c_str());
    sharedmem_img_subscription_ =
        this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            sharedmem_img_topic_name_, 10,
            std::bind(&CenterNet3DDetectionNode::SharedMemImgProcess, this,
                      std::placeholders::_1));
#else
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "Unsupport shared mem");
#endif
  } else {
    RCLCPP_WARN(rclcpp::get_logger("mono3d_indoor_detection"),
                "Create subscription with topic_name: %s",
                ros_img_topic_name_.c_str());
    ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        ros_img_topic_name_, 10,
        std::bind(&CenterNet3DDetectionNode::RosImgProcess, this,
                  std::placeholders::_1));
  }

  return 0;
}

int CenterNet3DDetectionNode::SetNodePara() {
  RCLCPP_INFO(rclcpp::get_logger("mono3d_indoor_detection"), "Set node para.");
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 2;
  return 0;
}

int CenterNet3DDetectionNode::SetOutputParser() {
  // set output parser
  auto model_manage = GetModel();
  if (!model_manage || !dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "Invalid model");
    return -1;
  }

  if (model_manage->GetOutputCount() < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "Error! Model %s output count is %d, unmatch with count %d",
                 dnn_node_para_ptr_->model_name.c_str(),
                 model_manage->GetOutputCount(), model_output_count_);
    return -1;
  }

  for (int i = 0; i < output_index_; ++i) {
    std::shared_ptr<OutputParser> assist_parser =
        std::make_shared<CenterNet3DAssistParser>();
    model_manage->SetOutputParser(i, assist_parser);
  }
  // set 3D paser
  auto output_desc = std::make_shared<OutputDescription>(
      model_manage, output_index_, "3D_branch");
  for (int i = 0; i < output_index_; ++i) {
    output_desc->GetDependencies().push_back(i);
  }
  output_desc->SetType("3D");
  model_manage->SetOutputDescription(output_desc);
  std::shared_ptr<OutputParser> out_parser =
      std::make_shared<CenterNet3DOutputParser>(config_file_path_);
  model_manage->SetOutputParser(output_index_, out_parser);

  return 0;
}

#define CV_DRAW_LINE(p0, p1) \
  cv::line(image, cv::Point(points[p0][0], points[p0][1]), \
  cv::Point(points[p1][0], points[p1][1]), \
      CV_RGB(0, 255, 0), 2);

#define CV_PUT_TEXT(text, px, py, offset) \
  { double fontScale = 3.0l; int thickness = 3u, baseline; \
    cv::Size text_size = cv::getTextSize(text, \
      cv::HersheyFonts::FONT_HERSHEY_PLAIN, fontScale, thickness, &baseline); \
    cv::Point point(px , py); \
    point.y += offset;\
    cv::Rect rect(point.x, point.y, \
                  text_size.width, text_size.height); \
    point.y += text_size.height;\
    offset += text_size.height;\
    cv::putText(image, text, point, \
      cv::HersheyFonts::FONT_HERSHEY_PLAIN, \
      fontScale, CV_RGB(0, 255, 128), thickness); }

// 使用算法输出进行渲染
void Render3DBox(const BBox3D &box, cv::Mat &image) {
  auto &points = box.corners2d_upscale;
  CV_DRAW_LINE(0, 1)
  CV_DRAW_LINE(0, 3)
  CV_DRAW_LINE(0, 4)
  CV_DRAW_LINE(1, 2)
  CV_DRAW_LINE(1, 5)
  CV_DRAW_LINE(2, 3)
  CV_DRAW_LINE(2, 6)
  CV_DRAW_LINE(3, 7)
  CV_DRAW_LINE(4, 5)
  CV_DRAW_LINE(4, 7)
  CV_DRAW_LINE(5, 6)
  CV_DRAW_LINE(6, 7)

  std::string box_type = std::to_string(box.class_label);
  switch (box.class_label) {
    case BBox3D::CHARGING_BASE:
      box_type = "charging_base";
      break;
    case BBox3D::SLIPPER:
      box_type = "slipper";
      break;
    case BBox3D::TRASH_CAN:
      box_type = "trash_can";
      break;
  }
  std::string score = std::to_string(box.score);
  int offset = 0;
  if (-M_PI_2 <= box.r || box.r <= M_PI_2) {
    CV_PUT_TEXT(box_type, points[7][0], points[7][1], offset);
    CV_PUT_TEXT(score, points[7][0], points[7][1], offset);
  } else {
    CV_PUT_TEXT(box_type, points[6][0], points[6][1], offset);
    CV_PUT_TEXT(score, points[6][0], points[6][1], offset);
  }
}

// 使用发布的msg进行渲染
void Render3DBox(const ai_msgs::msg::PerceptionTargets::UniquePtr &pub_data, cv::Mat &image) {
  auto draw_line = [&image](const ai_msgs::msg::Point& points, size_t p1, size_t p2){
    cv::line(image, cv::Point(points.point.at(p1).x, points.point.at(p1).y),
            cv::Point(points.point.at(p2).x, points.point.at(p2).y),
            CV_RGB(0, 255, 0), 2);
  };

  auto put_text = [&image](const std::string& text, int px, int py, int& offset) {
    double fontScale = 3.0l;
    int thickness = 3u, baseline;
    cv::Size text_size = cv::getTextSize(text, cv::HersheyFonts::FONT_HERSHEY_PLAIN, fontScale, thickness, &baseline);
    cv::Point point(px, py);
    point.y += offset;
    cv::Rect rect(point.x, point.y,
                  text_size.width, text_size.height);
    point.y += text_size.height;
    offset += text_size.height * 1.2;
    cv::putText(image, text, point,
      cv::HersheyFonts::FONT_HERSHEY_PLAIN,
      fontScale, CV_RGB(0, 255, 128), thickness);
  };

  for (const auto& target : pub_data->targets) {
    std::string score{""};
    float rotation = 0;
    for (const auto& attribute : target.attributes) {
      if (attribute.type == "score") {
        score = std::to_string(attribute.value);
      }
      if (attribute.type == "rotation") {
        rotation = attribute.value;
      }
    }

    for (const auto& point : target.points) {
      if ("corners" != point.type) {
        continue;
      }
      
      draw_line(point, 0, 1);
      draw_line(point, 0, 3);
      draw_line(point, 0, 4);
      draw_line(point, 1, 2);
      draw_line(point, 1, 5);
      draw_line(point, 2, 3);
      draw_line(point, 2, 6);
      draw_line(point, 3, 7);
      draw_line(point, 4, 5);
      draw_line(point, 4, 7);
      draw_line(point, 5, 6);
      draw_line(point, 6, 7);
      
      int offset = 0;
      if (-M_PI_2 <= rotation || rotation <= M_PI_2) {
        put_text(target.type, point.point[7].x, point.point[7].y, offset);
        put_text(score, point.point[7].x, point.point[7].y, offset);
      } else {
        put_text(target.type, point.point[6].x, point.point[6].y, offset);
        put_text(score, point.point[6].x, point.point[6].y, offset);
      }
    }
  }
}

int CenterNet3DDetectionNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput> &node_output) {
  if (!msg_publisher_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid msg_publisher_");
    return -1;
  }
  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);
  ai_msgs::msg::Perf perf;
  perf.set__type("PostProcess");
  perf.stamp_start.sec = time_start.tv_sec;
  perf.stamp_start.nanosec = time_start.tv_nsec;

  auto centernet_3d_output = std::dynamic_pointer_cast<CenterNet3DOutput>(node_output);
  if (centernet_3d_output) {
    std::stringstream ss;
    ss << "3D output dection info";
    if (centernet_3d_output->image_msg_header_) {
      ss << ", frame_id: " << centernet_3d_output->image_msg_header_->frame_id
         << ", stamp: " << centernet_3d_output->image_msg_header_->stamp.sec << "."
         << centernet_3d_output->image_msg_header_->stamp.nanosec;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("mono3d_indoor_detection"), "%s",
                ss.str().c_str());
  }

  const auto &outputs = node_output->outputs;
  RCLCPP_DEBUG(rclcpp::get_logger("mono3d_indoor_detection"), "outputs size: %d",
              outputs.size());
  if (outputs.empty() ||
      static_cast<int32_t>(outputs.size()) < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"), "Invalid outputs");
    return -1;
  }
  int smart_fps = 0;
  {
    auto tp_now = std::chrono::system_clock::now();
    output_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - output_tp_)
                        .count();
    if (interval >= 1000) {
      smart_fps = output_frameCount_;
      RCLCPP_INFO(rclcpp::get_logger("mono3d_indoor_detection"), "Smart fps = %d",
                  smart_fps);
      output_frameCount_ = 0;
      output_tp_ = std::chrono::system_clock::now();
    }
  }

  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());
  if (centernet_3d_output->image_msg_header_) {
    pub_data->header.set__stamp(centernet_3d_output->image_msg_header_->stamp);
    pub_data->header.set__frame_id(centernet_3d_output->image_msg_header_->frame_id);
  }

  pub_data->set__fps(smart_fps);
  auto *det_result =
      dynamic_cast<CenterNet3DDetResult *>(outputs[output_index_].get());
  if (!det_result) {
    RCLCPP_INFO(rclcpp::get_logger("mono3d_indoor_detection"), "invalid cast");
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("mono3d_indoor_detection"), "out box size: %d",
                det_result->boxes.size());

    std::map<std::string, ai_msgs::msg::Target> target_list;
    std::vector<ai_msgs::msg::Target> targets;
    targets.reserve(det_result->boxes.size());
    for(auto &box : det_result->boxes) {
      ai_msgs::msg::Target target;
      switch (box.class_label) {
        case BBox3D::CHARGING_BASE:
          target.type = "charging_base";
          break;
        case BBox3D::SLIPPER:
          target.type = "slipper";
          break;
        case BBox3D::TRASH_CAN:
          target.type = "trash_can";
          break;
      }
      RCLCPP_INFO(rclcpp::get_logger("mono3d_detection"), "target type: %s",
                  target.type.c_str());

      ai_msgs::msg::Attribute attribute;
      ai_msgs::msg::Point point;
      attribute.type = "width";
      attribute.value = box.w;
      RCLCPP_INFO(rclcpp::get_logger("mono3d_detection"),
                  "target type: %s, value: %f", attribute.type.c_str(),
                  attribute.value);
      target.attributes.push_back(attribute);
      attribute.type = "height";
      attribute.value = box.h;
      RCLCPP_INFO(rclcpp::get_logger("mono3d_detection"),
                  "target type: %s, value: %f", attribute.type.c_str(),
                  attribute.value);
      target.attributes.push_back(attribute);
      attribute.type = "length";
      attribute.value = box.l;
      RCLCPP_INFO(rclcpp::get_logger("mono3d_detection"),
                  "target type: %s, value: %f", attribute.type.c_str(),
                  attribute.value);
      target.attributes.push_back(attribute);
      attribute.type = "rotation";
      attribute.value = box.r;
      RCLCPP_INFO(rclcpp::get_logger("mono3d_detection"),
                  "target type: %s, value: %f", attribute.type.c_str(),
                  attribute.value);
      target.attributes.push_back(attribute);
      attribute.type = "x";
      attribute.value = box.x;
      RCLCPP_INFO(rclcpp::get_logger("mono3d_detection"),
                  "target type: %s, value: %f", attribute.type.c_str(),
                  attribute.value);
      target.attributes.push_back(attribute);
      attribute.type = "y";
      attribute.value = box.y;
      RCLCPP_INFO(rclcpp::get_logger("mono3d_detection"),
                  "target type: %s, value: %f", attribute.type.c_str(),
                  attribute.value);
      target.attributes.push_back(attribute);
      attribute.type = "z";
      attribute.value = box.z;
      RCLCPP_INFO(rclcpp::get_logger("mono3d_detection"),
                  "target type: %s, value: %f", attribute.type.c_str(),
                  attribute.value);
      target.attributes.push_back(attribute);
      attribute.type = "depth";
      attribute.value = box.d;
      RCLCPP_INFO(rclcpp::get_logger("mono3d_detection"),
                  "target type: %s, value: %f", attribute.type.c_str(),
                  attribute.value);
      target.attributes.push_back(attribute);
      attribute.type = "score";
      attribute.value = box.score;
      RCLCPP_INFO(rclcpp::get_logger("mono3d_detection"),
                  "target type: %s, value: %f", attribute.type.c_str(),
                  box.score);
      target.attributes.push_back(attribute);

      // 8 corners
      /*
                   4----------5    6----------7
                 /|         /|    /|         /|
               / |        / |   / |        / |
             /  |       /  |  /  |       /   |
            7---|------6   |5---|------4    |
            |   |      |   ||   |      |    |
            |   |      |   ||   |      |    |
            |   0------|---1|   2------|---3
            |  /       |  / |  /       |  /
            | /     ^  | /  | /     v  | /
            |/         |/   |/         |/
            3----------2    1----------0
        */
      // 找到最大包络框
      int x_min = -1;
      int x_max = 0;
      int y_min = -1;
      int y_max = 0;
      for (const auto &corners : box.corners2d_upscale) {
        geometry_msgs::msg::Point32 g_point;
        g_point.x = corners[0];
        g_point.y = corners[1];
        if (x_min < 0 || x_min > g_point.x) {
          x_min = g_point.x;
        }
        if (x_max < g_point.x) {
          x_max = g_point.x;
        }
        if (y_min < 0 || y_min > g_point.y) {
          y_min = g_point.y;
        }
        if (y_max < g_point.y) {
          y_max = g_point.y;
        }
        point.point.push_back(g_point);
      }
      point.type = "corners";
      target.points.push_back(point);
      
      ai_msgs::msg::Roi roi;
      roi.type = "corners";
      roi.rect.set__x_offset(x_min);
      roi.rect.set__y_offset(y_min);
      roi.rect.set__width(x_max - x_min);
      roi.rect.set__height(y_max - y_min);
      target.rois.emplace_back(roi);
      targets.push_back(target);
    }

    pub_data->targets = std::move(targets);

    // 使用算法输出进行渲染
    // if (!centernet_3d_output->image_name_.empty()) {
    //   auto img_bgr = cv::imread(centernet_3d_output->image_name_);
    //   for(auto &box : det_result->boxes) {
    //     Render3DBox(box, img_bgr);
    //   }
    //   std::string::size_type iPos =
    //           centernet_3d_output->image_name_.find_last_of('/') + 1;
    //   std::string filename =
    //           centernet_3d_output->image_name_.substr(
    //                   iPos, centernet_3d_output->image_name_.length() - iPos);
    //   cv::imwrite("./result/" + filename, img_bgr);
    // }
  }

  clock_gettime(CLOCK_REALTIME, &time_start);
  perf.stamp_end.sec = time_start.tv_sec;
  perf.stamp_end.nanosec = time_start.tv_nsec;
  pub_data->perfs.emplace_back(perf);

  // 使用发布的msg进行渲染
  if (dump_render_img_ && centernet_3d_output->mat_) {
    cv::Mat img_bgr = *centernet_3d_output->mat_;
    Render3DBox(pub_data, img_bgr);
    std::string filename = "./result/";
    if (centernet_3d_output->image_msg_header_) {
      filename += centernet_3d_output->image_msg_header_->frame_id + "_" +
                  std::to_string(centernet_3d_output->image_msg_header_->stamp.sec) + "_" +
                  std::to_string(centernet_3d_output->image_msg_header_->stamp.sec);
    }
    filename += ".jpeg";
    cv::imwrite(filename, img_bgr);
  }
  
  msg_publisher_->publish(std::move(pub_data));
  return 0;
}

int CenterNet3DDetectionNode::Predict(
    std::vector<std::shared_ptr<DNNInput>> &inputs,
    const std::shared_ptr<std::vector<hbDNNRoi>> rois,
    std::shared_ptr<DnnNodeOutput> dnn_output) {
  return Run(inputs, dnn_output, rois, false);
}

void CenterNet3DDetectionNode::RosImgProcess(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step
     << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "."
     << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("mono3d_indoor_detection"), "%s",
              ss.str().c_str());

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if ("rgb8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
    auto cv_img =
        cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_DEBUG(rclcpp::get_logger("mono3d_indoor_detection"),
                   "after cvtColorForDisplay cost ms: %d", interval);
    }

    pyramid = ImageUtils::GetNV12Pyramid(cv_img->image, model_input_height_,
                                         model_input_width_);
#else
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "Unsupport cv bridge");
#endif
  } else if ("nv12" == img_msg->encoding) {
    pyramid = ImageUtils::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height,
        img_msg->width, model_input_height_, model_input_width_);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "Get Nv12 pym fail");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono3d_indoor_detection"),
                 "after GetNV12Pyramid cost ms: %d", interval);
  }

  RCLCPP_INFO(rclcpp::get_logger("mono3d_indoor_detection"),
              "Dnn node begin to predict");
  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<CenterNet3DOutput>();
  dnn_output->src_img_width_ = img_msg->width;
  dnn_output->src_img_height_ = img_msg->height;
  dnn_output->image_msg_header_ = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header_->set__frame_id(img_msg->header.frame_id);
  dnn_output->image_msg_header_->set__stamp(img_msg->header.stamp);
  dnn_output->image_name_ = "";

  if (dump_render_img_ && ("nv12" == std::string(reinterpret_cast<const char *>(img_msg->encoding.data())))) {
    cv::Mat nv12(img_msg->height * 3 / 2, img_msg->width, CV_8UC1,
    const_cast<char*>(reinterpret_cast<const char *>(img_msg->data.data())));
    cv::Mat img_bgr;                               //  get bgr mat from pyramid
    cv::cvtColor(nv12, img_bgr, CV_YUV2BGR_NV12);  //  nv12 to bgr
    dnn_output->mat_ = std::make_shared<cv::Mat>(img_bgr);
  }

  // 3. 开始预测
  uint32_t ret = Predict(inputs, nullptr, dnn_output);
  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono3d_indoor_detection"),
                 "after Predict cost ms: %d", interval);
  }
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "predict img failed!");
  }
  return;
}

int CenterNet3DDetectionNode::PredictByImage(
        const std::string &image) {
  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  // bgr img，支持将图片resize到模型输入size
  pyramid = ImageUtils::GetNV12Pyramid(
          image, ImageType::BGR,
          model_input_height_, model_input_width_);
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "Get Nv12 pym fail with image: %s", image.c_str());
    return -1;
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<CenterNet3DOutput>();
  dnn_output->src_img_width_ = 1920;
  dnn_output->src_img_height_ = 1024;
  dnn_output->image_msg_header_ = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header_->set__frame_id("feedback");
  dnn_output->image_msg_header_->set__stamp(rclcpp::Time());
  dnn_output->image_name_ = image;
  // dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  // dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);

  if (dump_render_img_) {
    dnn_output->mat_ = std::make_shared<cv::Mat>(cv::imread(image, cv::IMREAD_COLOR));
  }

  // 3. 开始预测
  uint32_t ret = Predict(inputs, nullptr, dnn_output);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "predict img failed!");
  }
  return ret;
}

#ifdef SHARED_MEM_ENABLED
void CenterNet3DDetectionNode::SharedMemImgProcess(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("mono3d_indoor_detection"),
               "go into shared mem");

  // dump recved img msg
  // std::ofstream ofs("img_" + std::to_string(img_msg->index) + "." +
  // std::string(reinterpret_cast<const char*>(img_msg->encoding.data())));
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data_size);

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if ("nv12" ==
      std::string(reinterpret_cast<const char *>(img_msg->encoding.data()))) {
    pyramid = ImageUtils::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height,
        img_msg->width, model_input_height_, model_input_width_);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("mono3d_indoor_detection"),
                "share mem unsupported img encoding: %s", img_msg->encoding);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "Get Nv12 pym fail!");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono3d_indoor_detection"),
                 "after GetNV12Pyramid cost ms: %d", interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<CenterNet3DOutput>();
  dnn_output->src_img_width_ = img_msg->width;
  dnn_output->src_img_height_ = img_msg->height;
  dnn_output->image_msg_header_ = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header_->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header_->set__stamp(img_msg->time_stamp);

  if (dump_render_img_ && ("nv12" == std::string(reinterpret_cast<const char *>(img_msg->encoding.data())))) {
    cv::Mat nv12(img_msg->height * 3 / 2, img_msg->width, CV_8UC1,
    const_cast<char*>(reinterpret_cast<const char *>(img_msg->data.data())));
    cv::Mat img_bgr;                               //  get bgr mat from pyramid
    cv::cvtColor(nv12, img_bgr, CV_YUV2BGR_NV12);  //  nv12 to bgr
    dnn_output->mat_ = std::make_shared<cv::Mat>(img_bgr);
  }

  // 3. 开始预测
  int ret = Predict(inputs, nullptr, dnn_output);
  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono3d_indoor_detection"),
                 "after Predict cost ms: %d", interval);
  }

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "Run predict failed!");
  }
  return;
}
#endif
