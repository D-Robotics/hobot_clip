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

#include <fstream>
#include <iostream>
#include <string>

#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>

#include "include/clip_manage_node.h"
#include "include/database.h"
#include "include/feature_workflow.h"

void getImagesFromDirectory(const std::string& folder, std::vector<std::string>& images) {
    DIR* dir;
    struct dirent* entry;

    // Open the directory
    if (!(dir = opendir(folder.c_str()))) {
        RCLCPP_ERROR(rclcpp::get_logger("ClipNode"), "Cannot open directory: %s", folder.c_str()); 
        return;
    }

    while ((entry = readdir(dir)) != nullptr) {
        std::string path = folder + "/" + entry->d_name;

        // Skip "." and ".."
        if (entry->d_name[0] == '.') {
            continue;
        }

        struct stat info;
        if (stat(path.c_str(), &info) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("ClipNode"), "Cannot access %s", path.c_str()); 
            continue;
        }

        if (S_ISDIR(info.st_mode)) {
            // Recursively get images from subdirectories
            getImagesFromDirectory(path, images);
        } else if (S_ISREG(info.st_mode)) {
            // Get the file extension
            std::string extension = path.substr(path.find_last_of(".") + 1);
            // Convert the extension to lowercase
            std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
            // Check if the file has a valid image extension
            if (extension == "png" || extension == "jpg" || extension == "jpeg") {
                images.push_back(path);
            }
        }
    }
    closedir(dir);
}

ClipNode::ClipNode(const std::string &node_name, const NodeOptions &options)
    : rclcpp::Node(node_name, options), db() {
  this->declare_parameter<int>("mode", mode_);
  this->declare_parameter<std::string>("db_file", db_file_);
  this->declare_parameter<std::string>("result_folder", result_folder_);
  this->declare_parameter<std::string>("storage_folder", storage_folder_);
  this->declare_parameter<std::string>("text", text_);
  this->declare_parameter<int>("topk", topk_);

  this->get_parameter<int>("mode", mode_);
  this->get_parameter<std::string>("db_file", db_file_);
  this->get_parameter<std::string>("result_folder", result_folder_);
  this->get_parameter<std::string>("storage_folder", storage_folder_);
  this->get_parameter<std::string>("text", text_);
  this->get_parameter<int>("topk", topk_);

  std::stringstream ss;
  ss << "Parameter:"
     << "\n mode(0:storage, 1:query): " << mode_
     << "\n db file: " << db_file_
     << "\n storage folder: " << storage_folder_
     << "\n text: " << text_
     << "\n result folder: " << result_folder_
     << "\n topk: " << topk_;
  RCLCPP_WARN(rclcpp::get_logger("ClipNode"), "%s", ss.str().c_str());

  encode_image_client_ = std::make_shared<GetImageFeatureClient>();
  sp_task_image = 
    std::make_shared<std::thread>([this](){
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(encode_image_client_);
    exec.spin();
    });

  encode_text_client_ = std::make_shared<GetTextFeatureClient>();
  sp_task_text = 
    std::make_shared<std::thread>([this](){
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(encode_text_client_);
    exec.spin();
    });

  db.initialize(db_file_);
  if (!db.createTable()) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
      "Failed to create table.");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("ClipNode"), "ClipNode start."); 
  if (mode_ == 0) {
    Storage();
  } else {
    Run();
  }
}

ClipNode::~ClipNode() {
  if (sp_task_text && sp_task_text->joinable())
  {
    sp_task_text->join();
    sp_task_text.reset();
  }
  if (sp_task_image && sp_task_image->joinable())
  {
    sp_task_image->join();
    sp_task_image.reset();
  }
  rclcpp::shutdown();
}

int ClipNode::Run() {

  std::vector<std::string> texts;
  texts.push_back(text_);

  if (encode_text_client_) {

    encode_text_client_->set_feedback_callback(std::function<void(const std::shared_ptr<const clip_msgs::action::GetFeatures_Feedback_<std::allocator<void>>>)>(
      [&](const std::shared_ptr<const clip_msgs::action::GetFeatures_Feedback_<std::allocator<void>>> feedback) {
        // 处理接收到的反馈信息
        auto& item = feedback->item;

        ClipItem clipitem;
        clipitem.type = feedback->item.type;
        clipitem.url = feedback->item.url;
        clipitem.text = feedback->item.text;
        clipitem.feature.assign(feedback->item.feature.begin(), feedback->item.feature.end()); // 复制特征值
        clipitem.extra.assign(feedback->item.extra.begin(), feedback->item.extra.end());       // 复制额外信息
        
        const float* data_text = clipitem.feature.data();
        std::vector<ClipItem> target_items;

        // 获取开始时间
        auto start = std::chrono::high_resolution_clock::now();
        Query(data_text, target_items);
        
        // 获取结束时间
        auto end = std::chrono::high_resolution_clock::now();

        // 计算时间差
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        RCLCPP_WARN(rclcpp::get_logger("ClipNode"),
                "Query finished! Cost %d ms.", duration.count());

        // 定义目标目录
        std::string command1 = "rm -rf " + result_folder_;
        std::system(command1.c_str());
        std::string command2 = "mkdir -p " + result_folder_;
        std::system(command2.c_str());
        for (int i = 0; i < target_items.size(); i++) {
          auto& item = target_items[i];
          RCLCPP_WARN(rclcpp::get_logger("ClipNode"),
              "Query Result %s, similarity: %f", item.url.c_str(), item.similarity);
          std::string command = "ln -s " + item.url + " " + result_folder_ + "/" + std::to_string(i) + ".jpg";
          int result = std::system(command.c_str());
          if (result != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
              "Link file %s failed.", item.url);
          }
        }
      }));
    int ret = encode_text_client_->send_goal(texts, 10000);
  }
}


int ClipNode::Storage() {

  std::vector<std::string> urls;
  getImagesFromDirectory(storage_folder_, urls);
  
  // std::vector<std::string> target_urls;
  // for (const auto& url : urls) {
  //   if (!db.urlExists(url)) {
  //     target_urls.push_back(url);
  //   } 
  // }
  // swap(target_urls, urls);

  RCLCPP_INFO(rclcpp::get_logger("ClipNode"),
              "Storage size: %d.", urls.size());

  if (encode_image_client_) {

    encode_image_client_->set_feedback_callback(std::function<void(const std::shared_ptr<const clip_msgs::action::GetFeatures_Feedback_<std::allocator<void>>>)>(
      [&](const std::shared_ptr<const clip_msgs::action::GetFeatures_Feedback_<std::allocator<void>>> feedback) {
        // 处理接收到的反馈信息
        auto& item = feedback->item;

        std::stringstream ss;
        ss << "Received feedback: ["
          << feedback->current_progress << "]"
          << " url: " << feedback->item.url;
        RCLCPP_INFO(rclcpp::get_logger("ClipNode"), "%s", ss.str().c_str());

        ClipItem clipitem;
        clipitem.type = feedback->item.type;
        clipitem.url = feedback->item.url;

        clipitem.feature.assign(feedback->item.feature.begin(), feedback->item.feature.end()); // 复制特征值
        clipitem.extra.assign(feedback->item.extra.begin(), feedback->item.extra.end());       // 复制额外信息
        // std::cout << clipitem << std::endl;
        
        if (!db.insertItem(clipitem)) {
          RCLCPP_ERROR(rclcpp::get_logger("ClipNode"), "Failed to insert item.");
        }

      }));

    int ret = encode_image_client_->send_goal(urls, 10000);
  }

  RCLCPP_WARN(rclcpp::get_logger("ClipNode"),
            "Storage finish, current num of database: %d.",
            db.getItemCount());
}

// 定义函数，过滤出 type 为 false 的 ClipItem
int ClipNode::Query(const float *data,
              std::vector<ClipItem>& target_items) {

  int num = db.getItemCount();
  RCLCPP_WARN(rclcpp::get_logger("ClipNode"),
          "Query start, num of database: %d.", num);
  num = (num / 10) + 1;

  for (int i = 0; i < num; i++) {
    std::vector<ClipItem> image_items = db.queryItemsByPage(i, 10);
    for (auto& item : image_items) {
      if (!item.type) {
        continue;
      }
      const float* data_image = item.feature.data();
      item.similarity = cosine_similarity(data_image, data);
      target_items.push_back(item);
    }

    // 降序排序
    std::sort(target_items.begin(), target_items.end(), compareBySimilarity);

    // 检查向量大小并删除多余元素
    if (target_items.size() > topk_) {
      target_items.erase(target_items.begin() + topk_, target_items.end());
    }
  }
  return 0;
}