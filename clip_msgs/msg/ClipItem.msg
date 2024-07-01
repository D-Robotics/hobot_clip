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

# ClipItem.msg

# item 类型, true: 图片, false: 文本
bool type

# item 文本, 仅item为文本时有效
string text

# item 文件路径地址, 仅item为图片时有效
string url

# item 对应的特征值
float32[] feature

# item 额外信息
string[] extra