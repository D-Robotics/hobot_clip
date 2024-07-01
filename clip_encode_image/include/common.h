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

#include <iomanip>
#include <iostream>
#include <cmath>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "clip_msgs/msg/clip_item.hpp"

#ifndef COMMON_H_
#define COMMON_H_

struct ActionTaskRes {
  bool task_status_ = false;
  bool task_result_ = false;
};

struct ActionInputTaskInfo {

  // request
  bool type_ = true;
  std::vector<std::string> urls_;
};

using RequestTaskType =
  std::function<std::shared_ptr<ActionTaskRes>(std::shared_ptr<ActionInputTaskInfo>)>;

using CancelTaskType =
  std::function<bool()>;

#endif // COMMON_H_
