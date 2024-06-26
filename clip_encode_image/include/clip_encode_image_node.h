// Copyright (c) 2024，Horizon Robotics.
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

#include <memory>
#include <string>
#include <vector>

#include "ai_msgs/msg/perf.hpp"
#include "dnn_node/dnn_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#include "include/action_server.h"
#include "include/common.h"

#ifndef CLIP_ENCODE_IMAGE_NODE_H_
#define CLIP_ENCODE_IMAGE_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::ModelTaskType;

struct EncodeImageOutput : public DnnNodeOutput {
  std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;

  std::string url;  // encode 对应的url

  ai_msgs::msg::Perf perf_preprocess;
};

class ClipEncodeImageNode : public DnnNode {
 public:
  ClipEncodeImageNode(const std::string &node_name,
                 const NodeOptions &options = NodeOptions());
  ~ClipEncodeImageNode() override;

 protected:
  int SetNodePara() override;

  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  // 用于预测的图片来源，0：订阅到的image msg；1：本地nv12格式图片
  int feed_type_ = 1;
  std::string image_ = "config/CLIP.png";
  std::string model_file_name_ = "config/full_model_11.bin";
  std::string model_name_ = "full_model_11";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;

  int model_input_width_ = 224;
  int model_input_height_ = 224;
  int feature_size_ = 512;
  
  // 使用同步模式
  int is_sync_mode_ = 0;
  // 使用shared mem通信方式订阅图片
  int is_shared_mem_sub_ = 1;

  int FeedFromLocal(std::string& url);

  std::shared_ptr<std::thread> sp_task_;

#ifdef SHARED_MEM_ENABLED
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
  void SharedMemImgProcess(
      const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif

  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  // 目前只支持订阅原图，可以使用压缩图"/image_raw/compressed" topic
  // 和sensor_msgs::msg::CompressedImage格式扩展订阅压缩图
  std::string ros_img_sub_topic_name_ = "/image_raw";
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // 将订阅到的图片数据转成pym之后缓存
  // 在线程中执行推理，避免阻塞订阅IO通道，导致AI msg消息丢失
  std::mutex mtx_img_;
  std::condition_variable cv_img_;
  std::queue<std::string> urls_;

  void RunPredict();
  std::shared_ptr<std::thread> predict_task_ = nullptr;

  std::shared_ptr<ActionServer> action_server_ = nullptr;
  // 执行发出的请求
  std::shared_ptr<ActionTaskRes> OnRequest(std::shared_ptr<ActionInputTaskInfo>);
};

#endif  // CLIP_ENCODE_IMAGE_NODE_H_
