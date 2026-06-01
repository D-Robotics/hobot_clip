// Copyright (c) 2026，D-Robotics.
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

#include "include/image_proc.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#ifdef PLATFORM_X5
#include "dnn/hb_sys.h"
#endif
#include "rclcpp/rclcpp.hpp"

namespace hobot {
namespace dnn_node {

// ============================================================================
// X5 (libdnn) implementation
// ============================================================================
#if defined(PLATFORM_X5)

std::shared_ptr<DNNTensor> ImageProc::GetBGRTensorFromBGR(
    const std::string &image_file,
    int scaled_img_height,
    int scaled_img_width,
    hbDNNTensorProperties &tensor_properties,
    float &ratio,
    ImageType image_type) {
  auto w_stride = BPU_ALIGN(scaled_img_width);
  int channel = 3;

  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  int original_img_width = bgr_mat.cols;
  int original_img_height = bgr_mat.rows;

  cv::Mat pad_frame;
  if (static_cast<uint32_t>(original_img_width) != w_stride ||
      original_img_height != scaled_img_height) {
    pad_frame =
        cv::Mat(scaled_img_height, w_stride, CV_8UC3, cv::Scalar::all(0));
    if (static_cast<uint32_t>(original_img_width) > w_stride ||
        original_img_height > scaled_img_height) {
      float ratio_w =
          static_cast<float>(original_img_width) / static_cast<float>(w_stride);
      float ratio_h = static_cast<float>(original_img_height) /
                      static_cast<float>(scaled_img_height);
      float dst_ratio = std::max(ratio_w, ratio_h);
      ratio = dst_ratio;
      uint32_t resized_width =
          static_cast<float>(original_img_width) / dst_ratio;
      uint32_t resized_height =
          static_cast<float>(original_img_height) / dst_ratio;
      cv::resize(bgr_mat, bgr_mat, cv::Size(resized_width, resized_height));
    }

    bgr_mat.copyTo(pad_frame(
      cv::Rect((w_stride - bgr_mat.cols) / 2,
                (scaled_img_height - bgr_mat.rows) / 2,
                bgr_mat.cols,
                bgr_mat.rows)));

  } else {
    pad_frame = bgr_mat;
  }

  cv::Mat mat_tmp;
  int src_elem_size = 1;
  switch (tensor_properties.tensorType)
  {
    case HB_DNN_TENSOR_TYPE_F32: {
      src_elem_size = 4;
      pad_frame.convertTo(mat_tmp, CV_32F);
      mat_tmp /= 255.0;
    } break;
    default: RCLCPP_ERROR(rclcpp::get_logger("image_proc"),
          "Tensor Type %d is not support", tensor_properties.tensorType); break;
  }

  if (image_type == ImageType::RGB) {
    cv::cvtColor(mat_tmp, mat_tmp, cv::COLOR_BGR2RGB);
  }

  auto *mem = new hbSysMem;
  hbSysAllocCachedMem(mem, scaled_img_height * w_stride * channel * src_elem_size);
  uint8_t *data = mat_tmp.data;
  auto *hb_mem_addr = reinterpret_cast<uint8_t *>(mem->virAddr);

  for (int h = 0; h < scaled_img_height; ++h) {
    for (int w = 0; w < scaled_img_width; ++w) {
      for (int c = 0; c < channel; ++c) {
        auto *raw = hb_mem_addr + c * scaled_img_height * w_stride * src_elem_size + h * w_stride * src_elem_size + w * src_elem_size;
        auto *src = data + h * scaled_img_width * channel * src_elem_size + w * channel * src_elem_size + c * src_elem_size;
        memcpy(raw, src, src_elem_size);
      }
    }
  }

  hbSysFlushMem(mem, HB_SYS_MEM_CACHE_CLEAN);
  auto input_tensor = new DNNTensor;
  input_tensor->properties = tensor_properties;
  input_tensor->sysMem[0].virAddr = reinterpret_cast<void *>(mem->virAddr);
  input_tensor->sysMem[0].phyAddr = mem->phyAddr;
  input_tensor->sysMem[0].memSize = scaled_img_height * scaled_img_width * channel * src_elem_size;

  return std::shared_ptr<DNNTensor>(
      input_tensor, [mem](DNNTensor *input_tensor) {
        hbSysFreeMem(mem);
        delete mem;
        delete input_tensor;
      });
}

std::shared_ptr<DNNTensor> ImageProc::GetBGRTensorFromBGRImg(
    const char *in_img_data,
    const int &in_img_height,
    const int &in_img_width,
    const int &scaled_img_height,
    const int &scaled_img_width,
    hbDNNTensorProperties &tensor_properties) {
  auto w_stride = BPU_ALIGN(scaled_img_width);

  int src_elem_size = 1;
  switch (tensor_properties.tensorType)
  {
    case HB_DNN_TENSOR_TYPE_S8:
    case HB_DNN_TENSOR_TYPE_U8: src_elem_size = 1; break;
    case HB_DNN_TENSOR_TYPE_F16:
    case HB_DNN_TENSOR_TYPE_S16:
    case HB_DNN_TENSOR_TYPE_U16: src_elem_size = 2; break;
    case HB_DNN_TENSOR_TYPE_F32:
    case HB_DNN_TENSOR_TYPE_S32:
    case HB_DNN_TENSOR_TYPE_U32: src_elem_size = 4; break;
    case HB_DNN_TENSOR_TYPE_F64:
    case HB_DNN_TENSOR_TYPE_S64:
    case HB_DNN_TENSOR_TYPE_U64: src_elem_size = 8; break;
    default: RCLCPP_ERROR(rclcpp::get_logger("image_proc"),
        "Tensor Type %d is not support", tensor_properties.tensorType); break;
  }

  auto *mem = new hbSysMem;
  hbSysAllocCachedMem(mem, scaled_img_height * w_stride * 3 * src_elem_size);
  memset(mem->virAddr, 0, scaled_img_height * w_stride * 3 * src_elem_size);

  const uint8_t *data = reinterpret_cast<const uint8_t *>(in_img_data);
  auto *hb_mem_addr = reinterpret_cast<uint8_t *>(mem->virAddr);
  int copy_w = std::min(in_img_width, scaled_img_width);
  int copy_h = std::min(in_img_height, scaled_img_height);

  for (int h = 0; h < copy_h; ++h) {
    auto *raw = hb_mem_addr + h * w_stride * 3 * src_elem_size;
    auto *src = data + h * in_img_width * 3 * src_elem_size;
    memcpy(raw, src, copy_w * 3 * src_elem_size);
  }

  hbSysFlushMem(mem, HB_SYS_MEM_CACHE_CLEAN);
  auto input_tensor = new DNNTensor;

  input_tensor->properties = tensor_properties;
  input_tensor->sysMem[0].virAddr = reinterpret_cast<void *>(mem->virAddr);
  input_tensor->sysMem[0].phyAddr = mem->phyAddr;
  input_tensor->sysMem[0].memSize = scaled_img_height * scaled_img_width * 3 * src_elem_size;
  return std::shared_ptr<DNNTensor>(
      input_tensor, [mem](DNNTensor *input_tensor) {
        hbSysFreeMem(mem);
        delete mem;
        delete input_tensor;
      });
}

// ============================================================================
// S100 / UCP implementation (fallback)
// ============================================================================
#else

std::shared_ptr<DNNTensor> ImageProc::GetBGRTensorFromBGR(
    const std::string &image_file,
    int scaled_img_height,
    int scaled_img_width,
    hbDNNTensorProperties &tensor_properties,
    float &ratio,
    ImageType image_type) {
  auto w_stride = BPU_ALIGN(scaled_img_width);
  int channel = 3;

  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  int original_img_width = bgr_mat.cols;
  int original_img_height = bgr_mat.rows;

  cv::Mat pad_frame;
  if (static_cast<uint32_t>(original_img_width) != w_stride ||
      original_img_height != scaled_img_height) {
    pad_frame =
        cv::Mat(scaled_img_height, w_stride, CV_8UC3, cv::Scalar::all(0));
    if (static_cast<uint32_t>(original_img_width) > w_stride ||
        original_img_height > scaled_img_height) {
      float ratio_w =
          static_cast<float>(original_img_width) / static_cast<float>(w_stride);
      float ratio_h = static_cast<float>(original_img_height) /
                      static_cast<float>(scaled_img_height);
      float dst_ratio = std::max(ratio_w, ratio_h);
      ratio = dst_ratio;
      uint32_t resized_width =
          static_cast<float>(original_img_width) / dst_ratio;
      uint32_t resized_height =
          static_cast<float>(original_img_height) / dst_ratio;
      cv::resize(bgr_mat, bgr_mat, cv::Size(resized_width, resized_height));
    }

    bgr_mat.copyTo(pad_frame(
      cv::Rect((w_stride - bgr_mat.cols) / 2,
                (scaled_img_height - bgr_mat.rows) / 2,
                bgr_mat.cols,
                bgr_mat.rows)));
    
  } else {
    pad_frame = bgr_mat;
  }

  cv::Mat mat_tmp;
  int src_elem_size = 1;
  switch (tensor_properties.tensorType)
  {
    case HB_DNN_TENSOR_TYPE_U8: src_elem_size = 1; mat_tmp = pad_frame; break;
    case HB_DNN_TENSOR_TYPE_F32: {
      src_elem_size = 4;
      pad_frame.convertTo(mat_tmp, CV_32F);
      mat_tmp /= 255.0;
    } break;
    default: RCLCPP_ERROR(rclcpp::get_logger("image_proc"),
          "Tensor Type %d is not support", tensor_properties.tensorType); break;
  }

  if (image_type == ImageType::RGB) {
    cv::cvtColor(mat_tmp, mat_tmp, cv::COLOR_BGR2RGB);
  }

  auto *mem = new hbUCPSysMem;
  hbUCPMallocCached(mem, scaled_img_height * w_stride * channel * src_elem_size, 0);
  uint8_t *data = mat_tmp.data;
  auto *hb_mem_addr = reinterpret_cast<uint8_t *>(mem->virAddr);

  for (int h = 0; h < scaled_img_height; ++h) {
    for (int w = 0; w < scaled_img_width; ++w) {
      for (int c = 0; c < channel; ++c) {
        auto *raw = hb_mem_addr + c * scaled_img_height * w_stride * src_elem_size + h * w_stride * src_elem_size + w * src_elem_size;
        auto *src = data + h * scaled_img_width * channel * src_elem_size + w * channel * src_elem_size + c * src_elem_size;
        memcpy(raw, src, src_elem_size);
      }
    }
  }

  hbUCPMemFlush(mem, HB_SYS_MEM_CACHE_CLEAN);
  auto input_tensor = new DNNTensor;
  input_tensor->properties = tensor_properties;
  input_tensor->sysMem.virAddr = reinterpret_cast<void *>(mem->virAddr);
  input_tensor->sysMem.phyAddr = mem->phyAddr;
  input_tensor->sysMem.memSize = scaled_img_height * scaled_img_width * channel * src_elem_size;

  return std::shared_ptr<DNNTensor>(
      input_tensor, [mem](DNNTensor *input_tensor) {
        hbUCPFree(mem);
        delete mem;
        delete input_tensor;
      });
}

std::shared_ptr<DNNTensor> ImageProc::GetBGRTensorFromBGRImg(
    const char *in_img_data,
    const int &in_img_height,
    const int &in_img_width,
    const int &scaled_img_height,
    const int &scaled_img_width,
    hbDNNTensorProperties &tensor_properties) {
  auto w_stride = BPU_ALIGN(scaled_img_width);

  int src_elem_size = 1;
  switch (tensor_properties.tensorType)
  {
    case HB_DNN_TENSOR_TYPE_S8:
    case HB_DNN_TENSOR_TYPE_U8: src_elem_size = 1; break;
    case HB_DNN_TENSOR_TYPE_F16:
    case HB_DNN_TENSOR_TYPE_S16:
    case HB_DNN_TENSOR_TYPE_U16: src_elem_size = 2; break;
    case HB_DNN_TENSOR_TYPE_F32:
    case HB_DNN_TENSOR_TYPE_S32:
    case HB_DNN_TENSOR_TYPE_U32: src_elem_size = 4; break;
    case HB_DNN_TENSOR_TYPE_F64:
    case HB_DNN_TENSOR_TYPE_S64:
    case HB_DNN_TENSOR_TYPE_U64: src_elem_size = 8; break;
    default: RCLCPP_ERROR(rclcpp::get_logger("image_proc"),
        "Tensor Type %d is not support", tensor_properties.tensorType); break;
  }

  auto *mem = new hbUCPSysMem;
  hbUCPMallocCached(mem, scaled_img_height * w_stride * 3 * src_elem_size, 0);
  memset(mem->virAddr, 0, scaled_img_height * w_stride * 3 * src_elem_size);

  const uint8_t *data = reinterpret_cast<const uint8_t *>(in_img_data);
  auto *hb_mem_addr = reinterpret_cast<uint8_t *>(mem->virAddr);
  int copy_w = std::min(in_img_width, scaled_img_width);
  int copy_h = std::min(in_img_height, scaled_img_height);

  for (int h = 0; h < copy_h; ++h) {
    auto *raw = hb_mem_addr + h * w_stride * 3 * src_elem_size;
    auto *src = data + h * in_img_width * 3 * src_elem_size;
    memcpy(raw, src, copy_w * 3 * src_elem_size);
  }

  hbUCPMemFlush(mem, HB_SYS_MEM_CACHE_CLEAN);
  auto input_tensor = new DNNTensor;

  input_tensor->properties = tensor_properties;
  input_tensor->sysMem.virAddr = reinterpret_cast<void *>(mem->virAddr);
  input_tensor->sysMem.phyAddr = mem->phyAddr;
  input_tensor->sysMem.memSize = scaled_img_height * scaled_img_width * 3 * src_elem_size;
  return std::shared_ptr<DNNTensor>(
      input_tensor, [mem](DNNTensor *input_tensor) {
        hbUCPFree(mem);
        delete mem;
        delete input_tensor;
      });
}

#endif  // PLATFORM_S100

}  // namespace dnn_node
}  // namespace hobot
