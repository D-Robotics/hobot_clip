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

#ifndef CLIP_MANAGE_H_
#define CLIP_MANAGE_H_

#include <cstdlib>
#include <ctime>
#include <stack>

#include <rclcpp/rclcpp.hpp>

#include "include/database.h"
#include "include/item.h"
#include "include/image_action_client.h"
#include "include/text_action_client.h"

using rclcpp::NodeOptions;
using std::placeholders::_1;

class ClipNode : public rclcpp::Node
{
public:
  ClipNode(const std::string &node_name,
        const NodeOptions &options = NodeOptions());

  virtual ~ClipNode();

  int Run();

  int Query(const float *data,
            std::vector<ClipItem>& target_items);

  int Storage();

private:

  int mode_ = 0; // mode 为 0, 入库, mode 为 1, 查询
  std::string db_file_ = "clip.db";
  std::string text_ = "a diagram";
  std::string storage_folder_ = "/userdata/config";
  std::string result_folder_ = "/userdata/result";
  ClipItemDatabase db;

  int topk_ = 10;

  std::shared_ptr<GetImageFeatureClient> encode_image_client_ = nullptr;
  std::shared_ptr<GetTextFeatureClient> encode_text_client_ = nullptr;

  std::shared_ptr<std::thread> sp_task_image;
  std::shared_ptr<std::thread> sp_task_text;
};

#endif  // CLIP_MANAGE_H_