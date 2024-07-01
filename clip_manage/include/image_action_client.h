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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "clip_msgs/action/get_features.hpp"
#include "clip_msgs/msg/clip_item.hpp"

using rclcpp::NodeOptions;
using namespace std::placeholders;

class GetImageFeatureClient : public rclcpp::Node
{
public:
  using GetFeatures = clip_msgs::action::GetFeatures;
  using GoalHandleGetFeatures = rclcpp_action::ClientGoalHandle<GetFeatures>;

  static std::shared_ptr<GetImageFeatureClient> GetInstance() {
    static std::shared_ptr<GetImageFeatureClient> instance = std::make_shared<GetImageFeatureClient>();
    return instance;
  }

  void CancelAllGoals() {
    RCLCPP_WARN(this->get_logger(), "cancel all goals");
    client_ptr_->async_cancel_all_goals();
    std::unique_lock<std::mutex> lock(goal_response_mutex_);
    goal_response_success_ = false;
    goal_response_condition_.notify_all();
  }

  explicit GetImageFeatureClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  
  ~GetImageFeatureClient() {
    CancelAllGoals();
  }

  int send_goal(std::vector<std::string>& urls, int timeout_seconds = 1000);

  void set_feedback_callback(std::function<void(const std::shared_ptr<const GetFeatures::Feedback>)> callback) {
    feedback_callback_ = callback;
  }

  std::function<void(const std::shared_ptr<const GetFeatures::Feedback>)> feedback_callback_; // 反馈回调函数

private:
  rclcpp_action::Client<GetFeatures>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(GoalHandleGetFeatures::SharedPtr goal_handle);

  void feedback_callback(
    GoalHandleGetFeatures::SharedPtr,
    const std::shared_ptr<const GetFeatures::Feedback> feedback);

  void result_callback(const GoalHandleGetFeatures::WrappedResult & result);
  
  std::mutex goal_response_mutex_;
  bool goal_response_success_ = false;
  std::condition_variable goal_response_condition_;

};  // class GetImageFeatureClient