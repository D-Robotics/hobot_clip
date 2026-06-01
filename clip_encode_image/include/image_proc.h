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

#ifndef CLIP_ENCODE_IMAGE_PROC_H
#define CLIP_ENCODE_IMAGE_PROC_H

#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node_data.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_32(w) ALIGNED_2E(w, 32U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

#if defined(PLATFORM_X5)
#define BPU_ALIGN(value) ALIGN_16(value)
#elif defined(PLATFORM_S100)
#define BPU_ALIGN(value) ALIGN_32(value)
#elif defined(PLATFORM_S600)
#define BPU_ALIGN(value) ALIGN_64(value)
#else
#define BPU_ALIGN(value) ALIGN_32(value)
#endif

namespace hobot {
namespace dnn_node {

enum class ImageType { BGR = 0, RGB = 1 };

class ImageProc {
 public:
  static std::shared_ptr<DNNTensor> GetBGRTensorFromBGRImg(
      const char *in_img_data,
      const int &in_img_height,
      const int &in_img_width,
      const int &scaled_img_height,
      const int &scaled_img_width,
      hbDNNTensorProperties &tensor_properties);

  static std::shared_ptr<DNNTensor> GetBGRTensorFromBGR(
      const std::string &image_file,
      int scaled_img_height,
      int scaled_img_width,
      hbDNNTensorProperties &tensor_properties,
      float &ratio,
      ImageType image_type = ImageType::BGR);
};

}  // namespace dnn_node
}  // namespace hobot

#endif  // CLIP_ENCODE_IMAGE_PROC_H
