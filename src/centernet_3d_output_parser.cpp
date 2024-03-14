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

#include <fstream>
#include "include/centernet_3d_output_parser.h"

#include "dnn/hb_dnn_ext.h"
#include "include/image_utils.h"
#include "rclcpp/rclcpp.hpp"

int get_tensor_hwc_index(std::shared_ptr<DNNTensor> tensor, int *h_index,
                         int *w_index, int *c_index) {
  if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    *h_index = 1;
    *w_index = 2;
    *c_index = 3;
  } else if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    *c_index = 1;
    *h_index = 2;
    *w_index = 3;
  } else {
    return -1;
  }
  return 0;
}

int32_t CenterNet3DOutputParser::Parse(
    std::shared_ptr<CenterNet3DDetResult> &output,
    std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
    std::shared_ptr<OutputDescription> &output_descriptions,
    std::shared_ptr<DNNTensor> &output_tensor,
    std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
    std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
    std::vector<std::shared_ptr<DNNResult>> &depend_outputs) {
  if (output_descriptions) {
    RCLCPP_DEBUG(rclcpp::get_logger("3D_detection_parser"),
                 "type: %s, GetDependencies size: %d",
                 output_descriptions->GetType().c_str(),
                 output_descriptions->GetDependencies().size());
    if (!output_descriptions->GetDependencies().empty()) {
      RCLCPP_DEBUG(rclcpp::get_logger("3D_detection_parser"),
                   "Dependencies: %d",
                   output_descriptions->GetDependencies().front());
    }
  }
  if (depend_output_tensors.size() < 6) {
    RCLCPP_ERROR(rclcpp::get_logger("3D_detection_parser"),
                 "depend out tensor size invalid cast");
    return -1;
  }

  std::shared_ptr<CenterNet3DDetResult> result;
  if (!output) {
    result = std::make_shared<CenterNet3DDetResult>();
    result->Reset();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<CenterNet3DDetResult>(output);
    result->Reset();
  }

  int ret = PostProcess(depend_output_tensors, result->boxes);
  if (ret != 0) {
    RCLCPP_INFO(rclcpp::get_logger("3D_detection_parser"),
                "postprocess return error, code = %d", ret);
  }
  return ret;
}

static inline void ScaleCaliMatrix(std::vector<std::vector<float>> &intrin_mat,
                                   float ratio, float y_shift) {
  for (size_t row = 0; row < intrin_mat.size(); row++) {
    for (size_t col = 0; col < intrin_mat[row].size(); col++) {
      if (row == 0 && col == 0) {  // fu
        intrin_mat[row][col] /= ratio;
      } else if (row == 0 && col == 2) {  // cu
        intrin_mat[row][col] /= ratio;
      } else if (row == 1 && col == 1) {  // fv
        intrin_mat[row][col] /= ratio;
      } else if (row == 1 && col == 2) {  // cv
        intrin_mat[row][col] /= ratio;
        intrin_mat[row][col] -= y_shift;  // shoud be changed with ROI
      } else {
        // do nothing
      }
    }
  }
}

inline float GetFloatByInt(int32_t value, uint32_t shift) {
  float ret_x = value;
  if (value != 0) {
    int *ix = reinterpret_cast<int *>(&ret_x);
    (*ix) -= shift * 0x00800000;
  }
  return ret_x;
}

// 定点转浮点
// IN: src_ptr, bpu_model, out_index; OUT: dest_ptr
// 返回0：转换成功；返回-1，BPU输出是浮点数据，不需要转换
// 输出结果dest_ptr, 需要在函数外部由用户申请空间
inline int ConvertOutputToFloat(void *src_ptr, void *dest_ptr,
                                const std::shared_ptr<DNNTensor> &tensor,
                                int out_index) {
  if (tensor->properties.tensorType == HB_DNN_TENSOR_TYPE_F32) {
    std::cout << "BPU Output data_type is float32, no need convert to float";
    return -1;
  }
  auto &aligned_shape = tensor->properties.validShape;
  auto &real_shape = tensor->properties.alignedShape;
  auto out_elem_size = 1, float_elem_size = 4;
  if (tensor->properties.tensorType == HB_DNN_TENSOR_TYPE_S32 ||
      tensor->properties.tensorType == HB_DNN_TENSOR_TYPE_U32) {
    out_elem_size = 4;
  }
  auto shift = tensor->properties.shift.shiftData;

  uint32_t dst_n_stride = real_shape.dimensionSize[1] *
                          real_shape.dimensionSize[2] *
                          real_shape.dimensionSize[3] * float_elem_size;
  uint32_t dst_h_stride = real_shape.dimensionSize[2] *
                          real_shape.dimensionSize[3] * float_elem_size;
  uint32_t dst_w_stride = real_shape.dimensionSize[3] * float_elem_size;
  uint32_t src_n_stride = aligned_shape.dimensionSize[1] *
                          aligned_shape.dimensionSize[2] *
                          aligned_shape.dimensionSize[3] * out_elem_size;
  uint32_t src_h_stride = aligned_shape.dimensionSize[2] *
                          aligned_shape.dimensionSize[3] * out_elem_size;
  uint32_t src_w_stride = aligned_shape.dimensionSize[3] * out_elem_size;

  float tmp_float_value;
  int32_t tmp_int32_value;

  for (int nn = 0; nn < real_shape.dimensionSize[0]; nn++) {
    void *cur_n_dst = reinterpret_cast<int8_t *>(dest_ptr) + nn * dst_n_stride;
    void *cur_n_src = reinterpret_cast<int8_t *>(src_ptr) + nn * src_n_stride;
    for (int hh = 0; hh < real_shape.dimensionSize[1]; hh++) {
      void *cur_h_dst =
          reinterpret_cast<int8_t *>(cur_n_dst) + hh * dst_h_stride;
      void *cur_h_src =
          reinterpret_cast<int8_t *>(cur_n_src) + hh * src_h_stride;
      for (int ww = 0; ww < real_shape.dimensionSize[2]; ww++) {
        void *cur_w_dst =
            reinterpret_cast<int8_t *>(cur_h_dst) + ww * dst_w_stride;
        void *cur_w_src =
            reinterpret_cast<int8_t *>(cur_h_src) + ww * src_w_stride;
        for (int cc = 0; cc < real_shape.dimensionSize[3]; cc++) {
          void *cur_c_dst =
              reinterpret_cast<int8_t *>(cur_w_dst) + cc * float_elem_size;
          void *cur_c_src =
              reinterpret_cast<int8_t *>(cur_w_src) + cc * out_elem_size;
          if (out_elem_size == 4) {
            tmp_int32_value = *(reinterpret_cast<int32_t *>(cur_c_src));
            tmp_float_value = GetFloatByInt(tmp_int32_value, shift[cc]);
            *(reinterpret_cast<float *>(cur_c_dst)) = tmp_float_value;
          } else {
            tmp_int32_value = *(reinterpret_cast<int8_t *>(cur_c_src));
            tmp_float_value = GetFloatByInt(tmp_int32_value, shift[cc]);
            *(reinterpret_cast<float *>(cur_c_dst)) = tmp_float_value;
          }
        }
      }
    }
  }
  return 0;
}

void DumpFeatureMap(void *src_ptr, const std::shared_ptr<DNNTensor> &tensor,
                    int layer) {
  std::ofstream feature_out("feature_map_" + std::to_string(layer) + "_.txt",
                            std::ios::out);
  auto d = tensor->properties.alignedShape.dimensionSize;
  std::vector<float> feature_map(d[1] * d[2] * d[3]);
  ConvertOutputToFloat(src_ptr, feature_map.data(), tensor, layer);
  for (const auto feature : feature_map) {
    feature_out << feature << std::endl;
  }
}

inline void CenterNet3DOutputParser::ParameterInit(
    std::vector<std::shared_ptr<DNNTensor>> &tensors) {
  if (!is_parameter_init_) {

    cv::FileStorage fsSettings(yaml_file_.c_str(),
            cv::FileStorage::READ);
    if(fsSettings.isOpened()) {
#define GET_SEETING(key, value) { \
     if (!key.empty() && key.isReal()) { \
      value = key.real(); }}

      GET_SEETING(fsSettings["Camera1.fx"], kCalibMatrix_[0][0])
      GET_SEETING(fsSettings["Camera1.fy"], kCalibMatrix_[1][1])
      GET_SEETING(fsSettings["Camera1.cx"], kCalibMatrix_[0][2])
      GET_SEETING(fsSettings["Camera1.cy"], kCalibMatrix_[1][2])
    }

    std::cout << "iou_th: " << iou_th_ << std::endl;
    std::cout << "box_score_th: " << box_score_th_ << std::endl;
    box_log_score_th_ = -std::log(1.0f / box_score_th_ - 1.0f);
    std::cout << "box_log_score_th: " << box_log_score_th_ << std::endl;
    std::cout << "fx: " << kCalibMatrix_[0][0] << std::endl;
    std::cout << "cx: " << kCalibMatrix_[0][2] << std::endl;
    std::cout << "fy: " << kCalibMatrix_[1][1] << std::endl;
    std::cout << "cy: " << kCalibMatrix_[1][2] << std::endl;

    cam_focal_length_ = kCalibMatrix_[0][0];
    std::cout << "nms_size: " << nms_size_ << std::endl;
    std::cout << "model_focal_length: " << model_focal_length_ << std::endl;
    std::cout << "cam_focal_length: " << cam_focal_length_ << std::endl;
    focal_length_scale_ = cam_focal_length_ / model_focal_length_;

    int h_idx, w_idx, c_idx;
    get_tensor_hwc_index(tensors[0], &h_idx, &w_idx, &c_idx);
    model_output_width_ =
        tensors[0]->properties.validShape.dimensionSize[w_idx];
    model_output_height_ =
        tensors[0]->properties.validShape.dimensionSize[h_idx];
    pool_height_ = model_output_height_;
    pool_width_ = model_output_width_;
    std::cout << "model out width:" << model_output_width_
              << ", height:" << model_output_height_ << std::endl;
    size_t out_layer = tensors.size();
    output_shifts_.resize(out_layer);
    for (size_t layer = 0; layer < out_layer; layer++) {
      int h_idx, w_idx, c_idx;
      get_tensor_hwc_index(tensors[layer], &h_idx, &w_idx, &c_idx);
      std::cout << "model out hidx:" << h_idx << ", w_idx:" << w_idx
                << ", c_idx:" << c_idx << std::endl;
      aligned_channel_size_.push_back(
          tensors[layer]->properties.alignedShape.dimensionSize[c_idx]);
      aligned_width_size_.push_back(
          tensors[layer]->properties.alignedShape.dimensionSize[w_idx]);
      channel_size_.push_back(
          tensors[layer]->properties.validShape.dimensionSize[c_idx]);

      std::cout << "aligned_channel_size_ push:"
                << tensors[layer]->properties.alignedShape.dimensionSize[c_idx]
                << std::endl;
      std::cout << "aligned_width_size_ push:"
                << tensors[layer]->properties.alignedShape.dimensionSize[w_idx]
                << std::endl;
      std::cout << "channel_size_ push:"
                << tensors[layer]->properties.validShape.dimensionSize[c_idx]
                << std::endl;

      std::cout << "shiftData len:" << tensors[layer]->properties.shift.shiftLen
                << std::endl;
      // std::cout << "shiftData:" <<
      // tensors[layer]->properties.shift.shiftData<< std::endl;
      std::cout << "shiftData channel_size_:" << channel_size_[layer]
                << std::endl;
      output_shifts_[layer].assign(
          tensors[layer]->properties.shift.shiftData,
          tensors[layer]->properties.shift.shiftData + channel_size_[layer]);
      if (layer == 0) {
        down_ratio_ =
            static_cast<float>(model_input_width_) / aligned_width_size_[layer];
        output_scale_ =
            static_cast<float>(model_input_width_) / model_input_width_;
        down_ratio_ = 8;
        output_scale_ = 8;
        // output_scale_ =
        //     static_cast<float>(dnn_result.src_image_width) /
        //     model_input_width_;
        for (unsigned char channel_shift : output_shifts_[layer]) {
          box_score_init_.push_back(box_log_score_th_ - 0.05f);
          std::cout << "box score_init:" << box_score_init_.back() << std::endl;
          box_score_thres_.push_back(box_log_score_th_);
          std::cout << "box score:" << box_score_thres_.back() << std::endl;
        }
      }
    }
    //  ScaleCaliMatrix(kCalibMatrix_, output_scale_, 0.f);
    is_parameter_init_ = true;
  }
}

int inline CenterNet3DOutputParser::GetSpecFeatureMap(void *virAddr,
                                                      const BBox3D &box,
                                                      void **data, int layer) {
  *data =
      reinterpret_cast<int32_t *>(virAddr) +
      box.grid_y * aligned_width_size_[layer] * aligned_channel_size_[layer] +
      box.grid_x * aligned_channel_size_[layer];
  return 0;
}

int CenterNet3DOutputParser::PostProcess(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::vector<BBox3D> &boxes_3d) {
  std::vector<std::vector<float>> feature_map;
  ParameterInit(tensors);
  boxes_3d.clear();
  std::vector<BBox> boxes_2d;
  int32_t *data;
  size_t out_layer = tensors.size();
  feature_map.resize(out_layer);
  for (size_t layer = 0; layer < out_layer; layer++) {
    // flush mem
    if (!tensors[layer]) {
      std::cout << "tensor layout null, error." << std::endl;
      return -1;
    }
    hbSysFlushMem(&(tensors[layer]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

    if (layer == 0) {
      auto score_grater = [](const BBox3D &a, const BBox3D &b) {
        return a.score > b.score;
      };
      MaxPoolingRefine(boxes_3d, tensors[layer]->sysMem[0].virAddr, layer);
      std::stable_sort(boxes_3d.begin(), boxes_3d.end(), score_grater);
      if (nms_size_ < boxes_3d.size()) {
        boxes_3d.resize(nms_size_);
      }
      boxes_2d.resize(boxes_3d.size());
    } else {
      feature_map[layer].resize(boxes_3d.size() * channel_size_[layer]);
      for (size_t i_bbox = 0; i_bbox < boxes_3d.size(); i_bbox++) {
        GetSpecFeatureMap(tensors[layer]->sysMem[0].virAddr, boxes_3d[i_bbox],
                          reinterpret_cast<void **>(&data), layer);
        for (size_t channel = 0; channel < channel_size_[layer]; ++channel) {
          uint32_t offset = i_bbox * channel_size_[layer] + channel;
          feature_map[layer][offset] = GetFloatByInt(data[channel],
                  output_shifts_[layer][channel]);
        }
      }
    }
//    DumpFeatureMap(tensors[layer]->sysMem[0].virAddr,
//                    tensors[layer],
//                    layer);
  }

  ParseBBoxes(boxes_3d, boxes_2d, feature_map);
  std::vector<BBox> bbox2d_bev;
  Nms2D(boxes_3d, boxes_2d, true);
  ConvertCornerToStandupBox(boxes_3d, bbox2d_bev);
  NmsBev(boxes_3d, boxes_2d, true);
  for (const auto &box : boxes_3d) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("mono3d_indoor_detection"),
            " depth: " << box.d << " width: " << box.w
              << " height: " << box.h << " length: " << box.l
              << " rotate: " << box.r << " [x, y, z]: "
              << "[ " << box.x << ", " << box.y << ", " << box.z << "]"
              << " lable: " << box.class_label << " socre: " << box.score);
  }
  return 0;
}

void CenterNet3DOutputParser::Create3DBBox(std::vector<BBox3D> &boxes,
                                           uint32_t grid_x, uint32_t grid_y,
                                           float score, uint16_t class_label,
                                           uint32_t grid_idx) {
  BBox3D bbox3d;
  bbox3d.grid_x = grid_x;
  bbox3d.grid_y = grid_y;
  bbox3d.score = score;
  bbox3d.class_label = class_label;
  bbox3d.grid_idx = grid_idx;
  boxes.push_back(std::move(bbox3d));
}

int CenterNet3DOutputParser::Nms2D(std::vector<BBox3D> &bboxes3d,
                                   std::vector<BBox> &bboxes, bool suppress) {
  auto boxes_size = bboxes.size();
  std::vector<float> areas(boxes_size);
  std::vector<bool> skip(boxes_size, false);

  for (uint32_t i = 0; i < boxes_size; i++) {
    areas[i] = (bboxes[i].x2 - bboxes[i].x1) * (bboxes[i].y2 - bboxes[i].y1);
  }
  uint32_t count = 0;
  for (uint32_t i = 0; count < boxes_size && i < skip.size(); i++) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (!suppress) {
        if (bboxes[i].category_name != bboxes[j].category_name) {
          continue;
        }
      }
      float xx1 = std::max(bboxes[i].x1, bboxes[j].x1);
      float yy1 = std::max(bboxes[i].y1, bboxes[j].y1);
      float xx2 = std::min(bboxes[i].x2, bboxes[j].x2);
      float yy2 = std::min(bboxes[i].y2, bboxes[j].y2);
      float area_intersection = (xx2 - xx1) * (yy2 - yy1);
      bool area_intersection_valid = (area_intersection > 0) && (xx2 - xx1 > 0);
      if (area_intersection_valid) {
        float iou =
            area_intersection / (areas[j] + areas[i] - area_intersection);
        if (iou > iou_th_) {
          skip[j] = true;
        }
      }
    }
    bboxes3d[count] = bboxes3d[i];
    bboxes[count] = bboxes[i];
    ++count;
  }
  bboxes3d.resize(count);
  bboxes.resize(count);
  return 0;
}

// Results are stored in bbox3d_.
int CenterNet3DOutputParser::NmsBev(std::vector<BBox3D> &bboxes3d,
                                    std::vector<BBox> &bboxes2d_bev,
                                    bool suppress = false) {
  std::vector<float> areas(bboxes3d.size());
  std::vector<bool> skip(bboxes3d.size(), false);

  for (size_t i = 0; i < bboxes3d.size(); i++) {
    areas[i] = (bboxes2d_bev[i].x2 - bboxes2d_bev[i].x1) *
               (bboxes2d_bev[i].y2 - bboxes2d_bev[i].y1);
  }
  unsigned int count = 0;
  for (size_t i = 0; count < bboxes3d.size() && i < skip.size(); i++) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (!suppress) {
        if (bboxes2d_bev[i].category_name != bboxes2d_bev[j].category_name) {
          continue;
        }
      }
      float xx1 = std::max(bboxes2d_bev[i].x1, bboxes2d_bev[j].x1);
      float yy1 = std::max(bboxes2d_bev[i].y1, bboxes2d_bev[j].y1);
      float xx2 = std::min(bboxes2d_bev[i].x2, bboxes2d_bev[j].x2);
      float yy2 = std::min(bboxes2d_bev[i].y2, bboxes2d_bev[j].y2);
      float area_intersection = (xx2 - xx1) * (yy2 - yy1);
      bool area_intersection_valid = (area_intersection > 0) && (xx2 - xx1 > 0);
      if (area_intersection_valid) {
        float iou =
            area_intersection / (areas[j] + areas[i] - area_intersection);
        if (iou > beviou_th_) {
          skip[j] = true;
        }
      }
    }
    bboxes3d[count] = bboxes3d[i];
    ++count;
  }
  bboxes3d.resize(count);
  bboxes2d_bev.resize(count);
  return 0;
}

int CenterNet3DOutputParser::MaxPoolingRefine(std::vector<BBox3D> &boxes,
                                              void *heat_map, int layer) {
  int cls_align = aligned_channel_size_[layer];
  auto *hm = static_cast<int32_t *>(heat_map);
  size_t cls_size = channel_size_[layer];
  uint8_t *shift = output_shifts_[layer].data();
//  static int id = 0;
//  std::ofstream feature_out("heat_map_" + std::to_string(id++) + "_.txt",
//                            std::ios::out);
  for (size_t ph = 0; ph < pool_height_; ph++) {
    for (size_t pw = 0; pw < pool_width_; pw++) {
      uint32_t pool_index = ph * pool_width_ + pw;
      uint32_t argmax_cls = 0;
      float max_score_f = -10000;
      uint32_t ali_index = cls_align * ph * pool_width_ + cls_align * pw;
      for (size_t c = 0; c < cls_size; ++c) {
        float max_score = box_score_init_[c];
        float hm_score_f =
                GetFloatByInt(hm[ali_index + c], shift[c]);
//        feature_out << 1 / (1 + std::exp(-hm_score_f)) << " ";
        if (hm_score_f <= max_score) {
          continue;
        }
        int hstart = static_cast<int>(
            std::max(ph * pool_stride_ - 1.0f * pool_pad_, 0.0f));
        int wstart = static_cast<int>(
            std::max(pw * pool_stride_ - 1.0f * pool_pad_, 0.0f));
        int hend = static_cast<int>(std::min(hstart + 1.0f * pool_kernel_,
                                             1.0f * model_output_height_));
        int wend = static_cast<int>(
            std::min(wstart + 1.0f * pool_kernel_, 1.0f * model_output_width_));

        float cur_max_score = hm_score_f;
        for (int h = hstart; h < hend; ++h) {
          for (int w = wstart; w < wend; ++w) {
            uint32_t index =
                    cls_align * h * pool_width_ + cls_align * w + c;
            float hm_score = GetFloatByInt(hm[index], shift[c]);
            if (hm_score > cur_max_score) {
              cur_max_score = hm_score;
            }
          }
        }

        if (cur_max_score == hm_score_f) {
          if (max_score_f < cur_max_score) {
            max_score_f = cur_max_score;
            argmax_cls = static_cast<uint32_t>(c);
          }
        }
      }
      if (max_score_f > box_log_score_th_) {
        Create3DBBox(boxes, static_cast<uint32_t>(pw),
                     static_cast<uint32_t>(ph), max_score_f,
                     static_cast<uint16_t>(argmax_cls),
                     static_cast<uint32_t>(pool_index));
      }
    }
//    feature_out << std::endl;
  }
  return 0;
}

void CenterNet3DOutputParser::ParseBBoxes(
    std::vector<BBox3D> &bbox, std::vector<BBox> &bbox2d,
    std::vector<std::vector<float>> &feature_map) {
  float center_x, center_y, box_w, box_h;
  float depth;
  float alpha, rot_y;
  float *depthmap = feature_map[1].data(), *rot = feature_map[2].data(),
        *dim = feature_map[3].data(), *loc_reg = feature_map[4].data(),
        *wh = feature_map[5].data();
  std::vector<float> loc3d(3, 0.0f);
  for (size_t i_bbox = 0; i_bbox < bbox.size(); ++i_bbox) {
    bbox[i_bbox].score = 1.0f / (1.0f + std::exp(-bbox[i_bbox].score));
    center_x = static_cast<float>(bbox[i_bbox].grid_x) + 0.5f;
    center_x *= down_ratio_;
    center_y = static_cast<float>(bbox[i_bbox].grid_y) + 0.5f;
    center_y *= down_ratio_;
    depth = 1.0f / (1.0f / (1.0f + std::exp(-depthmap[i_bbox])) + 1e-6f) - 1.0f;
    depth *= focal_length_scale_;
    ProjLocTo3D(center_x, center_y, depth, loc3d);

    loc3d[1] += dim[3 * i_bbox] / 2.0f;
    loc3d[1] += loc_reg[2 * i_bbox + 1] * focal_length_scale_;
    loc3d[0] += loc_reg[2 * i_bbox] * focal_length_scale_;

    if (use_multibin_) {
      alpha = GetMultiBinAlpha(rot, static_cast<uint32_t>(i_bbox));
      rot_y = GetMultiBinRotY(alpha, center_x);
    } else {
      alpha = GetSimpleAlpha(rot, static_cast<uint32_t>(i_bbox));
      rot_y = GetSimpleRotY(alpha, loc3d);
    }
    bbox[i_bbox].x = loc3d[0];
    bbox[i_bbox].y = loc3d[1];
    bbox[i_bbox].z = loc3d[2];
    bbox[i_bbox].l = dim[3 * i_bbox];
    bbox[i_bbox].w = dim[3 * i_bbox + 1];
    bbox[i_bbox].h = dim[3 * i_bbox + 2];
    bbox[i_bbox].d = depth;
    bbox[i_bbox].r = rot_y;

    box_w = wh[2 * i_bbox] * down_ratio_;
    box_h = wh[2 * i_bbox + 1] * down_ratio_;
    bbox2d[i_bbox].x1 = (center_x - box_w / 2.f);
    bbox2d[i_bbox].y1 = (center_y - box_h / 2.f + image_shift_);
    bbox2d[i_bbox].x2 = (center_x + box_w / 2.f);
    bbox2d[i_bbox].y2 = (center_y + box_h / 2.f + image_shift_);
    bbox2d[i_bbox].score = bbox[i_bbox].score;
    bbox2d[i_bbox].category_name = std::to_string(bbox[i_bbox].class_label);
    Get3DBboxCorners(loc3d, bbox[i_bbox]);
    ProjectToImage(bbox[i_bbox]);
  }
}

void CenterNet3DOutputParser::ProjectToImage(BBox3D &bbox) {
  float fx = kCalibMatrix_[0][0];
  float fy = kCalibMatrix_[1][1];
  float cx = kCalibMatrix_[0][2];
  float cy = kCalibMatrix_[1][2];
  float x, y;
  for (size_t row = 0; row < 8; row++) {
    float depth = bbox.corners3d[row][2];
    if (depth < 0) {
      // std::cout << "depth: " << depth << std::endl;
      depth = 0.001f;
    }
    // x = bbox->corners3d[row][0] / bbox->corners3d[row][2];
    // y = bbox->corners3d[row][1] / bbox->corners3d[row][2];
    x = bbox.corners3d[row][0] / depth;
    y = bbox.corners3d[row][1] / depth;

    // 512p resolution, undistort
    bbox.corners2d[row][0] = fx * x + cx;
    bbox.corners2d[row][1] = fy * y + cy;

    // rescale to target output resolution, undistort
    bbox.corners2d_upscale[row][0] = bbox.corners2d[row][0];
    bbox.corners2d_upscale[row][1] =
        (bbox.corners2d[row][1] + image_shift_);
  }
}

void CenterNet3DOutputParser::ProjLocTo3D(float center_x, float center_y,
                                          float depth,
                                          std::vector<float> &loc3d) {
  float x, y, z;
  z = depth - kCalibMatrix_[2][3];
  x = (center_x * depth - kCalibMatrix_[0][3] - kCalibMatrix_[0][2] * z) /
      kCalibMatrix_[0][0];
  y = (center_y * depth - kCalibMatrix_[1][3] - kCalibMatrix_[1][2] * z) /
      kCalibMatrix_[1][1];
  loc3d[0] = x;
  loc3d[1] = y;
  loc3d[2] = z;
}

inline float CenterNet3DOutputParser::GetMultiBinAlpha(float *rot, uint32_t idx,
                                                       uint32_t rot_dim) {
  float angle;
  idx *= rot_dim;
  if (rot[idx + 1] > rot[idx + 5]) {
    angle = atan(rot[idx + 2] / rot[idx + 3]) - 0.5f * M_PI;
  } else {
    angle = atan(rot[idx + 6] / rot[idx + 7]) + 0.5f * M_PI;
  }
  return angle;
}

inline float CenterNet3DOutputParser::GetSimpleAlpha(float *rot, uint32_t idx,
                                                     uint32_t rot_dim) {
  float angle;
  idx *= rot_dim;
  angle = atan2(rot[idx], rot[idx + 1]) - M_PI_2;
  return angle;
}

inline float CenterNet3DOutputParser::GetMultiBinRotY(float alpha, float cx) {
  float rot_y;
  rot_y = alpha + std::atan2(cx - kCalibMatrix_[0][2], kCalibMatrix_[0][0]);
  if (rot_y > M_PI) {
    rot_y -= 2.0f * M_PI;
  } else if (rot_y < -M_PI) {
    rot_y += 2.0f * M_PI;
  }
  return rot_y;
}

inline float CenterNet3DOutputParser::GetSimpleRotY(float alpha,
                                                    std::vector<float> &loc) {
  float rot_y;
  rot_y = alpha + atan2(loc[0], loc[2]);
  if (rot_y > M_PI) {
    rot_y -= 2.0f * M_PI;
  } else if (rot_y < -M_PI) {
    rot_y += 2.0f * M_PI;
  }
  return rot_y;
}

// Covert rotated 2D bbox to axis-aligned bbox (covex hull)
void CenterNet3DOutputParser::ConvertCornerToStandupBox(
    std::vector<BBox3D> &bbox, std::vector<BBox> &bbox2d) {
  bbox2d.resize(bbox.size());
  for (size_t i = 0; i < bbox.size(); i++) {
    // x_top_left = min(x1, x2, x3, x4)
    bbox2d[i].x1 = std::min({bbox[i].corners3d[0][0], bbox[i].corners3d[1][0],
                             bbox[i].corners3d[2][0], bbox[i].corners3d[3][0]},
                            std::less<float>());

    // y_top_left = min(y1, y2, y3, y4)
    bbox2d[i].y1 = std::min({bbox[i].corners3d[0][2], bbox[i].corners3d[1][2],
                             bbox[i].corners3d[2][2], bbox[i].corners3d[3][2]},
                            std::less<float>());

    // x_bottom_right = max(x1, x2, x3, x4)
    bbox2d[i].x2 = std::max({bbox[i].corners3d[0][0], bbox[i].corners3d[1][0],
                             bbox[i].corners3d[2][0], bbox[i].corners3d[3][0]},
                            std::less<float>());

    // y_bottom_right = max(y1, y2, y3, y4)
    bbox2d[i].y2 = std::max({bbox[i].corners3d[0][2], bbox[i].corners3d[1][2],
                             bbox[i].corners3d[2][2], bbox[i].corners3d[3][2]},
                            std::less<float>());

    bbox2d[i].score = bbox[i].score;
    bbox2d[i].category_name = std::to_string(bbox[i].class_label);
  }
}

void CenterNet3DOutputParser::Get3DBboxCorners(std::vector<float> &loc3d,
                                               BBox3D &bbox) {
  float c, s;
  c = std::cos(bbox.r);
  s = std::sin(bbox.r);
  float l = bbox.l, w = bbox.w, h = bbox.h;
  std::vector<float> rot_matrix = {c, 0.0f, s, 0.0f, 1.0f, 0.0f, -s, 0.0f, c};
  std::vector<float> x_corners = {l / 2, l / 2, -l / 2, -l / 2,
                                  l / 2, l / 2, -l / 2, -l / 2};
  std::vector<float> y_corners = {h / 2, h / 2, h / 2, h / 2,
                                  -h / 2, -h / 2, -h / 2, -h / 2};
  std::vector<float> z_corners = {w / 2, -w / 2, -w / 2, w / 2,
                                  w / 2, -w / 2, -w / 2, w / 2};

  for (size_t col = 0; col < 3; col++) {
    for (size_t row = 0; row < 8; row++) {
      bbox.corners3d[row][col] = rot_matrix[3 * col + 0] * x_corners[row] +
                                 rot_matrix[3 * col + 1] * y_corners[row] +
                                 rot_matrix[3 * col + 2] * z_corners[row] +
                                 loc3d[col];
    }
  }
}
