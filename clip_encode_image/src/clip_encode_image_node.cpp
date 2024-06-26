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

#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "rclcpp/rclcpp.hpp"

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "include/action_server.h"
#include "include/clip_encode_image_node.h"
#include "include/common.h"

builtin_interfaces::msg::Time ConvertToRosTime(
    const struct timespec& time_spec) {
  builtin_interfaces::msg::Time stamp;
  stamp.set__sec(time_spec.tv_sec);
  stamp.set__nanosec(time_spec.tv_nsec);
  return stamp;
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time& start,
                      const builtin_interfaces::msg::Time& end) {
  return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 -
         start.nanosec / 1000 / 1000;
}

ClipEncodeImageNode::ClipEncodeImageNode(const std::string& node_name,
                               const NodeOptions& options)
    : DnnNode(node_name, options) {
  this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<std::string>("image", image_);
  this->declare_parameter<std::string>("model_file_name", model_file_name_);

  this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<std::string>("image", image_);
  this->get_parameter<std::string>("model_file_name", model_file_name_);

  std::stringstream ss;
  ss << "Parameter:"
     << "\n feed_type(0:loacl, 1:sub): " << feed_type_
     << "\n is_shared_mem_sub: " << is_shared_mem_sub_
     << "\n is_sync_mode_: " << is_sync_mode_
     << "\n image: " << image_
     << "\n model_file_name_: " << model_file_name_;
  RCLCPP_WARN(rclcpp::get_logger("ClipImageNode"), "%s", ss.str().c_str());

  action_server_ = std::make_shared<ActionServer>();
  action_server_->SetActionProcess(
    std::bind(&ClipEncodeImageNode::OnRequest, this, std::placeholders::_1)
    );
  sp_task_ = 
    std::make_shared<std::thread>([this](){
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(action_server_);
    exec.spin();
    });
  
  // 使用基类接口初始化，加载模型
  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipImageNode"), "Init failed!");
    rclcpp::shutdown();
    return;
  }

  if (feed_type_ == 0) {
    FeedFromLocal(image_);
  } else {
    predict_task_ = std::make_shared<std::thread>(
        std::bind(&ClipEncodeImageNode::RunPredict, this));

    if (is_shared_mem_sub_) {
#ifdef SHARED_MEM_ENABLED
      RCLCPP_WARN(rclcpp::get_logger("ClipImageNode"),
                  "Create hbmem_subscription with topic_name: %s",
                  sharedmem_img_topic_name_.c_str());
      sharedmem_img_subscription_ =
          this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
              sharedmem_img_topic_name_,
              rclcpp::SensorDataQoS(),
              std::bind(&ClipEncodeImageNode::SharedMemImgProcess,
                        this,
                        std::placeholders::_1));
#else
      RCLCPP_ERROR(rclcpp::get_logger("ClipImageNode"), "Unsupport shared mem");
#endif
    } else {
      RCLCPP_WARN(rclcpp::get_logger("ClipImageNode"),
                  "Create subscription with topic_name: %s",
                  ros_img_sub_topic_name_.c_str());
      ros_img_subscription_ =
          this->create_subscription<sensor_msgs::msg::Image>(
              ros_img_sub_topic_name_,
              10,
              std::bind(
                  &ClipEncodeImageNode::RosImgProcess, this, std::placeholders::_1));
    }
  }
}

ClipEncodeImageNode::~ClipEncodeImageNode() {
  std::unique_lock<std::mutex> lg(mtx_img_);
  cv_img_.notify_all();
  lg.unlock();

  if (sp_task_ && sp_task_->joinable())
  {
    rclcpp::shutdown();
    sp_task_->join();
    sp_task_.reset();
  }

  if (predict_task_ && predict_task_->joinable()) {
    predict_task_->join();
    predict_task_.reset();
  }
}

int ClipEncodeImageNode::SetNodePara() {
  RCLCPP_INFO(rclcpp::get_logger("ClipImageNode"), "Set node para.");
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 4;
  return 0;
}

int ClipEncodeImageNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput>& node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }

  if (!node_output) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipImageNode"), "Invalid node output");
    return -1;
  }

  // 记录后处理开始时间
  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  auto encode_image_output = std::dynamic_pointer_cast<EncodeImageOutput>(node_output);
  if (!encode_image_output) {
    return -1;
  }

  // 1. 解析模型输出向量
  hbSysFlushMem(&(encode_image_output->output_tensors[0]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  float *data = reinterpret_cast<float *>(encode_image_output->output_tensors[0]->sysMem[0].virAddr);
  std::vector<float> float_vector(feature_size_);
  std::copy(data, data + feature_size_, float_vector.begin());

  // 2. 服务模式下, 将结果返回给客户端
  if (action_server_ != nullptr) {
    clip_msgs::msg::ClipItem clipitem = clip_msgs::msg::ClipItem();
    clipitem.type = true;
    clipitem.url = encode_image_output->url;
    clipitem.feature = std::move(float_vector);
    action_server_->SetItemResult(clipitem);
  }

  // 3. 计算推理耗时
  // 填充perf性能统计信息, 前处理统计
  ai_msgs::msg::Perf perf_preprocess = std::move(encode_image_output->perf_preprocess);
  perf_preprocess.set__time_ms_duration(CalTimeMsDuration(
      perf_preprocess.stamp_start, perf_preprocess.stamp_end));

  // dnn node有输出统计信息
  if (node_output->rt_stat) {
    struct timespec time_now = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_now);

    // 推理统计
    ai_msgs::msg::Perf perf;
    perf.set__type(model_name_ + "_predict_infer");
    perf.stamp_start =
        ConvertToRosTime(node_output->rt_stat->infer_timespec_start);
    perf.stamp_end = ConvertToRosTime(node_output->rt_stat->infer_timespec_end);
    perf.set__time_ms_duration(node_output->rt_stat->infer_time_ms);

    perf.set__type(model_name_ + "_predict_parse");
    perf.stamp_start =
        ConvertToRosTime(node_output->rt_stat->parse_timespec_start);
    perf.stamp_end = ConvertToRosTime(node_output->rt_stat->parse_timespec_end);
    perf.set__time_ms_duration(node_output->rt_stat->parse_time_ms);

    // 后处理统计
    ai_msgs::msg::Perf perf_postprocess;
    perf_postprocess.set__type(model_name_ + "_postprocess");
    perf_postprocess.stamp_start = ConvertToRosTime(time_start);
    clock_gettime(CLOCK_REALTIME, &time_now);
    perf_postprocess.stamp_end = ConvertToRosTime(time_now);
    perf_postprocess.set__time_ms_duration(CalTimeMsDuration(
        perf_postprocess.stamp_start, perf_postprocess.stamp_end));

    // 如果当前帧有更新统计信息，输出统计信息
    if (node_output->rt_stat->fps_updated) {
      RCLCPP_WARN(rclcpp::get_logger("ClipImageNode"),
                  "Sub img fps: %.2f, Smart fps: %.2f, preprocess time ms: %d, infer time ms: %d, "
                  "post process time ms: %d",
                  node_output->rt_stat->input_fps,
                  node_output->rt_stat->output_fps,
                  static_cast<int>(perf_preprocess.time_ms_duration),
                  node_output->rt_stat->infer_time_ms,
                  static_cast<int>(perf_postprocess.time_ms_duration));
    }
  }

  return 0;
}


void ClipEncodeImageNode::RosImgProcess(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  auto model = GetModel();
  hbDNNTensorProperties tensor_properties;
  model->GetInputTensorProperties(tensor_properties, 0);

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step
     << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "_"
     << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("ClipImageNode"), "%s", ss.str().c_str());
  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<DNNTensor> tensor = nullptr;
  if ("jpg" == img_msg->encoding) {
    tensor = hobot::dnn_node::ImageProc::GetBGRTensorFromBGRImg(
        reinterpret_cast<const char*>(img_msg->data.data()),
        model_input_height_,
        model_input_width_,
        model_input_height_,
        model_input_width_,
        tensor_properties);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ClipImageNode"), "Unsupport img encoding: %s",
    img_msg->encoding.data());
    return;
  }
  if (!tensor) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipImageNode"), "Get nv12 tensor");
    return;
  }
  std::vector<std::shared_ptr<DNNTensor>> inputs;
  inputs.push_back(tensor);

  // 2. 创建推理输出数据
  auto dnn_output = std::make_shared<EncodeImageOutput>();
  // 将图片消息的header填充到输出数据中，用于表示推理输出对应的输入信息
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->image_msg_header->set__stamp(img_msg->header.stamp);
  // 将当前的时间戳填充到输出数据中，用于计算perf
  dnn_output->perf_preprocess.stamp_start.sec = time_start.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_start.tv_nsec;
  dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");

  // 3. 开始预测
  if (Run(inputs, dnn_output) != 0) {
    return;
  }    
}

#ifdef SHARED_MEM_ENABLED
void ClipEncodeImageNode::SharedMemImgProcess(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  auto model = GetModel();
  hbDNNTensorProperties tensor_properties;
  model->GetInputTensorProperties(tensor_properties, 0);

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  std::stringstream ss;
  ss << "Recved img encoding: "
     << std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step << ", index: " << img_msg->index
     << ", stamp: " << img_msg->time_stamp.sec << "_"
     << img_msg->time_stamp.nanosec << ", data size: " << img_msg->data_size;
  RCLCPP_INFO(rclcpp::get_logger("ClipImageNode"), "%s", ss.str().c_str());

  // 1. 将图片处理成模型输入数据类型DNNTensor
  std::shared_ptr<DNNTensor> tensor = nullptr;
  if ("jpg" == 
      std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
    std::shared_ptr<DNNTensor> tensor = nullptr;
    tensor = hobot::dnn_node::ImageProc::GetBGRTensorFromBGRImg(
        reinterpret_cast<const char*>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        img_msg->height,
        img_msg->width,
        tensor_properties);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ClipImageNode"),
                "Unsupported img encoding: %s",
                img_msg->encoding);
    return;
  }
  if (!tensor) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipImageNode"), "Get tensor fail!");
    return;
  }
  std::vector<std::shared_ptr<DNNTensor>> inputs;
  inputs.push_back(tensor);

  // 2. 创建推理输出数据
  auto dnn_output = std::make_shared<EncodeImageOutput>();
  // 将图片消息的header填充到输出数据中，用于表示推理输出对应的输入信息
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);
  // 将当前的时间戳填充到输出数据中，用于计算perf
  dnn_output->perf_preprocess.stamp_start.sec = time_start.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_start.tv_nsec;
  dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");

  uint32_t ret = 0;
  // 3. 开始预测
  if (Run(inputs, dnn_output) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipImageNode"), "Run Model Error");
    return;
  }

}
#endif

int ClipEncodeImageNode::FeedFromLocal(std::string& url) {
  if (access(url.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipImageNode"),
                 "Image: %s not exist!",
                 url.c_str());
    return -1;
  }

  auto model = GetModel();
  hbDNNTensorProperties tensor_properties;
  model->GetInputTensorProperties(tensor_properties, 0);

  auto dnn_output = std::make_shared<EncodeImageOutput>();
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_start.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_start.nanosec = time_now.tv_nsec;

  std::shared_ptr<DNNTensor> tensor = nullptr;
  tensor = hobot::dnn_node::ImageProc::GetBGRTensorFromBGR(url,
      model_input_height_, model_input_width_, tensor_properties);

  if (!tensor) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipImageNode"),
                 "Get tensor fail with image: %s",
                 url.c_str());
    return -1;
  }

  // 2. 使用图片数据创建DNNTensor
  // inputs将会作为模型的输入通过InferTask接口传入
  std::vector<std::shared_ptr<DNNTensor>> inputs;
  inputs.push_back(tensor);
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->perf_preprocess.stamp_end.sec = time_now.tv_sec;
  dnn_output->perf_preprocess.stamp_end.nanosec = time_now.tv_nsec;
  dnn_output->perf_preprocess.set__type(model_name_ + "_preprocess");
  dnn_output->url = url;

  // 3. 开始预测
  if (Run(inputs, dnn_output) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipImageNode"), "Run Model Error");
    return -1;
  }

  return 0;
}

void ClipEncodeImageNode::RunPredict() {

  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lg(mtx_img_);
    cv_img_.wait(lg, [this]() { return !urls_.empty() || !rclcpp::ok(); });
    if (urls_.empty()) {
      continue;
    }
    if (!rclcpp::ok()) {
      break;
    }
    std::string& url = urls_.front();
    FeedFromLocal(url);
    urls_.pop();
    lg.unlock();
  }
}

std::shared_ptr<ActionTaskRes> ClipEncodeImageNode::OnRequest(std::shared_ptr<ActionInputTaskInfo> task_info) {
  std::shared_ptr<ActionTaskRes> task_res = std::make_shared<ActionTaskRes>();
  if (!task_info) return task_res;
  if (!task_info->type_) {
    task_res->task_result_ = false;
    return task_res;
  }

  std::stringstream ss;
  ss << "run task, type: " << task_info->type_
     << " , urls length: " << task_info->urls_.size();
  RCLCPP_INFO(rclcpp::get_logger("ClipImageNode"), "%s", ss.str().c_str());    

  bool type = task_info->type_;
  auto urls = task_info->urls_;
  
  task_res->task_result_ = true;
  // 将准备好的输入输出数据存进缓存
  std::unique_lock<std::mutex> lg(mtx_img_);
  for (auto& url : task_info->urls_) {
    urls_.push(url);
  }
  cv_img_.notify_one();
  lg.unlock();
  return task_res;
}