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

#include "include/item.h"
#include <iostream>

// 比较函数，用于按 similarity 降序排序
bool compareBySimilarity(const ClipItem& a, const ClipItem& b) {
    return a.similarity > b.similarity;
}

// Overload operator<< for ClipItem
std::ostream& operator<<(std::ostream& os, const ClipItem& item) {
    os << "ID: " << item.id << "\n"
       << "Timestamp: " << item.timestamp << "\n"
       << "Type: " << (item.type ? "Image" : "Text") << "\n"
       << "Name: " << item.name << "\n"
       << "Text: " << item.text << "\n"
       << "URL: " << item.url << "\n"
       << "Feature: ";
    for (const auto& f : item.feature) {
        os << f << " ";
    }
    os << "\nExtra: ";
    for (const auto& e : item.extra) {
        os << e << " ";
    }
    os << "\n----------------------------------------\n";
    return os;
}