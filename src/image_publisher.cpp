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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <memory>

#include <dirent.h>
#include "dnn_node/util/image_proc.h"

void load_images(const std::string &image_directory,
                 std::vector<std::string> &images) {
  DIR *dir;
  struct dirent *ptr;
  if ((dir = opendir(image_directory.c_str())) == nullptr) {
    std::cout << "Open dir error...\n";
    return;
  }
  while ((ptr = readdir(dir)) != nullptr) {
    if (strcmp(ptr->d_name, ".") == 0 ||
        strcmp(ptr->d_name, "..") == 0) {
      continue;
    } else if (ptr->d_type == 8) {
      images.push_back(image_directory + ptr->d_name);
    } else if (ptr->d_type == 10 || ptr->d_type == 4) {
      continue;
    }
  }
  closedir(dir);
}

sensor_msgs::msg::Image::SharedPtr create_image(
        const std::string &image) {
  sensor_msgs::msg::Image::SharedPtr out_img =
          std::make_shared<sensor_msgs::msg::Image>();
  cv::Mat nv_12;
  cv::Mat bgr_image = cv::imread(image, cv::IMREAD_COLOR);
  hobot::dnn_node::ImageProc::BGRToNv12(bgr_image, nv_12);
  out_img->encoding = "nv12";
  out_img->width = nv_12.cols;
  out_img->height = nv_12.rows * 2 / 3;
  int image_size = nv_12.cols * nv_12.rows;
  out_img->data.resize(image_size);
  std::memcpy(out_img->data.data(), nv_12.data, image_size);
  out_img->header.frame_id = image;  // "camera";
  out_img->header.stamp = rclcpp::Clock().now();
  return out_img;
}

void pub_images(rclcpp::Node::SharedPtr node,
        const std::string &image_directory) {
  RCLCPP_INFO(node->get_logger(), "images directory: %s", image_directory.c_str());
  auto image_publisher = node->create_publisher<sensor_msgs::msg::Image>(
          "/image_raw", 5);
  std::vector<std::string> images;
  load_images(image_directory, images);
  auto last_pub = std::chrono::high_resolution_clock::now();
  for (const auto &image : images) {
    auto out_img = create_image(image);
    auto duration_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    last_pub - std::chrono::high_resolution_clock::now()).count();
    if (duration_ms < 33) {
      std::this_thread::sleep_for(
              std::chrono::milliseconds(33 - duration_ms));
    }
    last_pub = std::chrono::high_resolution_clock::now();
    image_publisher->publish(*out_img);
    RCLCPP_INFO(node->get_logger(), "publish image: %s", image.c_str());
  }
}

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  if (argc < 2) {
    RCUTILS_LOG_ERROR(
            "image_publisher requires directory. Typical command-line usage:\n"
            "\t$ ros2 run mono3d_indoor_detection image_publisher <directory>");
    return -1;
  }

  pub_images(std::make_shared<rclcpp::Node>("image_publisher"),
          std::string(argv[1]));
}