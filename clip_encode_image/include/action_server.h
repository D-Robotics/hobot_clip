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

#include <memory>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "clip_msgs/action/get_features.hpp"
#include "clip_msgs/msg/clip_item.hpp"

#include "include/common.h"

using rclcpp::NodeOptions;

#ifndef ACTION_SERVER_H_
#define ACTION_SERVER_H_

class ActionServer : public rclcpp::Node
{
public:
  using GetFeatures = clip_msgs::action::GetFeatures;
  using GoalHandleGetFeatures = rclcpp_action::ServerGoalHandle<GetFeatures>;

  explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ActionServer() {}

  void SetActionProcess(RequestTaskType request_task) {
    request_task_ = request_task;
  }

  void SetItemResult(const clip_msgs::msg::ClipItem& clipitem) {
    std::unique_lock<std::mutex> lg(mtx_);
    clipitems_.push(clipitem);
    cv_.notify_one();
    lg.unlock();
  }

  void Cancel() {
    cancel_requested_ = true;
  }

private:
  rclcpp_action::Server<GetFeatures>::SharedPtr action_server_;
  std::atomic_bool is_running_;

  RequestTaskType request_task_ = nullptr;
  std::queue<clip_msgs::msg::ClipItem> clipitems_;

  std::shared_ptr<ActionInputTaskInfo> task_info_ = std::make_shared<ActionInputTaskInfo>();

  std::mutex mtx_;
  std::condition_variable cv_;

  std::atomic_bool cancel_requested_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GetFeatures::Goal> goal)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Received goal request with type: " << goal->type);
    (void)uuid;

    if (is_running_) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Action server is running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    is_running_ = true;
    cancel_requested_ = false;

    task_info_->type_ = goal->type;
    task_info_->urls_ = goal->urls;
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGetFeatures> goal_handle)
  {
    RCLCPP_WARN(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGetFeatures> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGetFeatures> goal_handle);

};  // class ActionServer

#endif // ACTION_SERVER_H_