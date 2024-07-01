import numpy as np
import onnxruntime as rt
import rclpy
import time

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from clip_msgs.action import GetFeatures
from clip_msgs.msg import ClipItem

import clip_encode_text.clip as clip

class ClipEncodeTextNode(Node):
    def __init__(self):
        super().__init__('clip_encode_text_node')
        self.get_logger().warn('Clip Encode Text Node has been started.')
        
        # Declare parameters
        self.declare_parameter('feed_type', True)
        self.declare_parameter('dump_result', False)
        self.declare_parameter('model_file_name', "config/text_encoder.onnx")

        # Get parameter values
        self.feed_type = self.get_parameter('feed_type').get_parameter_value().bool_value
        self.dump_result = self.get_parameter('dump_result').get_parameter_value().bool_value
        self.model_file_name = self.get_parameter('model_file_name').get_parameter_value().string_value

        self.sess = rt.InferenceSession(self.model_file_name)
        if not self.feed_type:
            texts = ["a diagram", "a dog", "a cat"]
            self.runonnx(texts)
        else:
            self.action_server = ActionServer(
                self,
                GetFeatures,
                'clip_text_action',
                execute_callback=self.execute_callback,
                callback_group=ReentrantCallbackGroup(),
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback
            )

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        
        return GoalResponse.ACCEPT
        # return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal')

        # Access the goal
        request_type = goal_handle.request.type
        if request_type:
            self.get_logger().error('Get error type, request texts not images')
            return GetFeatures.Result()
        texts = goal_handle.request.texts
        self.get_logger().warn('Request texts: {0}'.format(texts))

        feedback_msg = GetFeatures.Feedback()
        num = len(texts)
        for i in range(num):
          if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return GetFeatures.Result()

          temptexts = []
          temptexts.append(texts[i])
          text_features_np = self.runonnx(temptexts)

          clipitem = ClipItem()
          clipitem.type = False
          clipitem.text = texts[i]
          clipitem.feature = text_features_np[0].tolist()

          feedback_msg.item = clipitem
          feedback_msg.current_progress = (i / num)
          goal_handle.publish_feedback(feedback_msg)

        result = GetFeatures.Result()
        result.success = True
        goal_handle.succeed()
        self.get_logger().warn('Clip Encode Text Node work success.')

        return result

    def runonnx(self, texts):
        start_time = time.time()
        
        # 1. simple tokenizer
        text_np = clip.tokenize(texts)

        # record time
        end_time = time.time()
        elapsed_time1 = end_time - start_time

        # 2. model inference 
        input_name_0 = self.sess.get_inputs()[0].name
        output_name_0 = self.sess.get_outputs()[0].name
        text_features_np = self.sess.run([output_name_0],{input_name_0:text_np})[0]
    
        # record time
        end_time = time.time()
        elapsed_time2 = end_time - start_time

        self.get_logger().info('clip input: {0}, token time: {1:.3f} s, inference time: {2:.3f} s'.format(texts, elapsed_time1, elapsed_time2))

        # 3. dump result
        if self.dump_result:
            texts_str = ','.join(texts)
            with open(texts_str + '.bin', 'wb') as f:
                text_features_np.tofile(f)

        return text_features_np

def main(args=None):
    rclpy.init(args=args)
    node = ClipEncodeTextNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()