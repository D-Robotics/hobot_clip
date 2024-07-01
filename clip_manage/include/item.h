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

#ifndef ITEM_H_
#define ITEM_H_

#include <iostream>
#include <vector>

struct ClipItem {
  bool type;        // item 类型, true: 图片, false: 文本
  std::string name; // item 名
  std::string text; // item text, 仅item为文本时有效
  std::string url;  // item 文件路径地址, 仅item为图片时有效
  std::vector<float> feature; // item 对应的特征, 长度512
  std::vector<std::string> extra;   // item 额外信息, 如对应深度图路径、IMU数据路径、其他说明等。

  int id;           // item 唯一id, 入库自动获取
  long timestamp;   // item 入库时间戳, 入库自动获取
  float similarity; // item 的相似度, 不入库
};

bool compareBySimilarity(const ClipItem& a, const ClipItem& b);

std::ostream& operator<<(std::ostream& os, const ClipItem& item);



#endif  // ITEM_H_