# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.node import Node

from clip_msgs.action import GetFeatures
from clip_msgs.msg import ClipItem

class ClipClientNode(Node):

    def __init__(self, query_type, num):
        super().__init__('client')
        self.type = query_type
        self.num = num
        if self.type:
            self.action_name = "/clip_image_action"
        else:
            self.action_name = "/clip_text_action"
        self.get_logger().info(self.action_name + " action start.")
        self.action_client = ActionClient(self, GetFeatures, self.action_name)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return

        self.goal_handle = goal_handle
        self.get_logger().info('Goal accepted!')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

        # Start a 2 second timer
        # self._timer = self.create_timer(2.0, self.timer_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeaded! Result: {0}'.format(result.success))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback current_progress: {0}'.format(feedback.feedback.current_progress))
        self.get_logger().info('Received feedback clipitem:')
        self.get_logger().info('[type]: {0}'.format('Image' if feedback.feedback.item.type else 'Text'))
        self.get_logger().info('[text]: {0}'.format(feedback.feedback.item.text))
        self.get_logger().info('[url]: {0}'.format(feedback.feedback.item.url))
        self.get_logger().info('[extra]: {0}'.format(feedback.feedback.item.extra))
        print("feature: ", feedback.feedback.item.feature)

    def timer_callback(self):
        print("cancel goal")
        self.cancel_goal()
        # Cancel the timer
        self._timer.cancel()

    def cancel_goal(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        goal_msg = GetFeatures.Goal()
        goal_msg.type = self.type
        texts = ["a diagram", "a dog", "a cat"]
        urls = []
        for i in range(self.num):
            urls.append("config/CLIP.png")
        goal_msg.texts = texts
        goal_msg.urls = urls

        self.get_logger().info('Sending goal request...')

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_image', action='store_true', help='action type.')
    parser.add_argument('--num', type=int, default=10, help='action num.')
    cfgs = parser.parse_args()

    rclpy.init(args=args)
    action_client = ClipClientNode(cfgs.use_image, cfgs.num)
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()