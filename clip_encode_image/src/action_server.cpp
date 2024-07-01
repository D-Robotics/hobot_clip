// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "include/action_server.h"
#include "include/common.h"

ActionServer::ActionServer(const rclcpp::NodeOptions & options)
  : Node("encode_image_server", options),
  is_running_(false),
  cancel_requested_(false)
{
  RCLCPP_INFO(this->get_logger(), "ActionServer is creating.");
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<GetFeatures>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "clip_image_action",
      std::bind(&ActionServer::handle_goal, this, _1, _2),
      std::bind(&ActionServer::handle_cancel, this, _1),
      std::bind(&ActionServer::handle_accepted, this, _1));
  RCLCPP_INFO(this->get_logger(), "ActionServer is created. action name is [clip_image_action]");
}

void ActionServer::execute(const std::shared_ptr<GoalHandleGetFeatures> goal_handle)
{
  is_running_ = true;
  RCLCPP_WARN(this->get_logger(), "Executing goal");

  if (!task_info_) {
    RCLCPP_ERROR(this->get_logger(), "Invalid task info");
    is_running_ = false;
    return;
  }

  bool task_result = false;
  if (request_task_) {
    std::shared_ptr<ActionTaskRes> task_res = request_task_(task_info_);
    if (task_res) {
      task_result = task_res->task_result_;
    }
  }

  const auto goal = goal_handle->get_goal();
  std::queue<std::string> urls;
  for (const auto& url : goal->urls) {
    urls.push(url);
  }
  float num = urls.size();
  auto result = std::make_shared<GetFeatures::Result>();
  while(!urls.empty()) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      is_running_ = false;
      RCLCPP_WARN(this->get_logger(), "Goal canceled");
      return;
    }

    if (cancel_requested_) {
      result->success = false;
      goal_handle->abort(result);
      is_running_ = false;
      RCLCPP_WARN(this->get_logger(), "Goal abort");
      cancel_requested_ = false;
      return;
    }

    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this]() { return !clipitems_.empty(); });  // 等待直到队列非空
    urls.pop();
    auto& item = clipitems_.front();
    auto feedback = std::make_shared<GetFeatures::Feedback>();
    feedback->item = std::move(item);
    feedback->current_progress = 1 - static_cast<float>(urls.size()) / num;
    goal_handle->publish_feedback(feedback);
    clipitems_.pop();
    lock.unlock();
    RCLCPP_INFO(this->get_logger(), "Publish feedback");
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    result->error_code = 0;
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_WARN_STREAM(this->get_logger(), "Goal complete, task_result: " << task_result);
  }
  is_running_ = false;
}
