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

#include "include/image_action_client.h"

GetImageFeatureClient::GetImageFeatureClient(const rclcpp::NodeOptions & options)
  : Node("image_action_client", options)
{
  this->client_ptr_ = rclcpp_action::create_client<GetFeatures>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "clip_image_action");
}

int GetImageFeatureClient::send_goal(std::vector<std::string>& urls, 
                        int timeout_seconds) {
    
  RCLCPP_WARN(this->get_logger(), "Action client recved goal");
  std::unique_lock<std::mutex> lock(goal_response_mutex_);
  RCLCPP_WARN(this->get_logger(), "Action client got lock");

  this->goal_response_success_ = false;

  if (!this->client_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    return -1;
  }

  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(timeout_seconds))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    this->goal_response_success_ = false;
    return -1;
  }

  auto goal_msg = GetFeatures::Goal();
  goal_msg.type = true;
  goal_msg.urls = std::move(urls);

  RCLCPP_WARN_STREAM(this->get_logger(), "Sending goal, type: "
    << goal_msg.type << ", urls size: " << urls.size());

  auto send_goal_options = rclcpp_action::Client<GetFeatures>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&GetImageFeatureClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&GetImageFeatureClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&GetImageFeatureClient::result_callback, this, _1);

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  
  if (goal_response_condition_.wait_for(lock, std::chrono::seconds(timeout_seconds)) == std::cv_status::timeout) {
    RCLCPP_ERROR(this->get_logger(), "Recv goal response timeout(%d seconds)!", timeout_seconds);
    CancelAllGoals();
    lock.unlock();
    return -1;
  } else {
    lock.unlock();
    if (goal_response_success_) {
      return 0;
    } else {
      return -1;
    }
  }
  return 0;
}

void GetImageFeatureClient::goal_response_callback(GoalHandleGetFeatures::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    std::unique_lock<std::mutex> lock(goal_response_mutex_);
    goal_response_success_ = false;
    goal_response_condition_.notify_all();
    lock.unlock();
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void GetImageFeatureClient::feedback_callback(
  GoalHandleGetFeatures::SharedPtr,
  const std::shared_ptr<const GetFeatures::Feedback> feedback)
{
  // 调用设置的回调函数
  if (feedback_callback_) {
    feedback_callback_(feedback);
  }
}

void GetImageFeatureClient::result_callback(const GoalHandleGetFeatures::WrappedResult & result)
{
  bool succseed = false;
  std::stringstream ss;
  ss << "Get Result errorcode: " << result.result->error_code;
  RCLCPP_WARN(this->get_logger(), "%s", ss.str().c_str());
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal was Successful!!!");
      succseed = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      succseed = false;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      succseed = false;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      succseed = false;
      break;
  }

  std::unique_lock<std::mutex> lock(goal_response_mutex_);
  goal_response_success_ = succseed;
  goal_response_condition_.notify_one();
  lock.unlock();
}