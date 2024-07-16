# Copyright (c) 2024，Horizon Robotics.
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

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    index_path = os.path.join(
        get_package_prefix('clip_manage'),
        "lib/clip_manage/config/")
    cp_cmd = "cp -r " + index_path + "index.html ."
    print("cp_cmd is ", cp_cmd)
    os.system(cp_cmd)

    # args that can be set from the command line or a default will be used
    mode_launch_arg = DeclareLaunchArgument(
        "clip_mode", default_value=TextSubstitution(text="0")
    )
    db_file_launch_arg = DeclareLaunchArgument(
        "clip_db_file", default_value=TextSubstitution(text="clip.db")
    )
    text_launch_arg = DeclareLaunchArgument(
        "clip_text", default_value=TextSubstitution(text="a diagram")
    )
    storage_folder_launch_arg = DeclareLaunchArgument(
        "clip_storage_folder", default_value=TextSubstitution(text="/userdata/config")
    )
    result_folder_launch_arg = DeclareLaunchArgument(
        "clip_result_folder", default_value=TextSubstitution(text="result")
    )

    # recorder node 示例节点pkg
    clip_encode_image = Node(
        package='clip_encode_image',
        executable='clip_encode_image',
        output='screen',
        parameters=[
            {"feed_type": 1},
            {"is_sync_mode": 1},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    clip_encode_text = Node(
        package='clip_encode_text',
        executable='clip_encode_text_node',
        output='screen',
        parameters=[
            {"feed_type": True},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    clip_manage = Node(
        package='clip_manage',
        executable='clip_manage',
        output='screen',
        parameters=[
            {"mode": LaunchConfiguration('clip_mode')},
            {"db_file": LaunchConfiguration('clip_db_file')},
            {"text": LaunchConfiguration('clip_text')},
            {"storage_folder": LaunchConfiguration('clip_storage_folder')},
            {"result_folder": LaunchConfiguration('clip_result_folder')},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        mode_launch_arg,
        db_file_launch_arg,
        text_launch_arg,
        storage_folder_launch_arg,
        result_folder_launch_arg,
        # clip encode image 编码服务节点pkg
        clip_encode_image,
        # clip encode text 编码服务节点pkg
        clip_encode_text,
        # clip manage 客户端节点pkg
        clip_manage,
    ])