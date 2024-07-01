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

#ifndef DATABASE_H_
#define DATABASE_H_

#include <iostream>
#include <sqlite3.h>
#include <string>
#include <vector>

#include "include/item.h"

class ClipItemDatabase {
public:
  // // 构造函数，初始化数据库名称
  // ClipItemDatabase(const std::string& db_name);
  ClipItemDatabase() : db_name(""), db(nullptr) {}
  explicit ClipItemDatabase(const std::string& db_name) : db_name(db_name), db(nullptr) {
    openDatabase(db_name);
  }

  // 析构函数，清理数据库资源
  ~ClipItemDatabase();

  void initialize(const std::string& db_name);

  // 创建表格，返回操作是否成功
  bool createTable();

  // 插入剪贴板项，返回操作是否成功
  bool insertItem(const ClipItem& item);

  // 删除指定ID的剪贴板项，返回操作是否成功
  bool deleteItem(int id);

  // 查询指定ID的剪贴板项，返回ClipItem对象
  ClipItem queryItem(int id);

  // 获取剪贴板项的总数量
  int getItemCount();

  // 分页查询剪贴板项，返回当前页的ClipItem对象列表
  std::vector<ClipItem> queryItemsByPage(int page_number, int page_size);

  // 检查指定URL是否存在，返回布尔值
  bool urlExists(const std::string& url);

private:
  // SQLite数据库连接对象
  sqlite3* db;
  // 数据库名称
  std::string db_name;

  void openDatabase(const std::string& db_name);

  // 执行SQL语句，返回操作是否成功
  bool executeSQL(const char* sql);

  // 检查剪贴板项是否存在，返回布尔值
  bool itemExists(const ClipItem& item);

  // 验证剪贴板项是否合法，返回布尔值
  bool isValidItem(const ClipItem& item);

  // 将字符串向量连接成一个字符串，使用指定的分隔符
  std::string join(const std::vector<std::string>& vec, const std::string& delimiter);

  // 将字符串按指定分隔符拆分成字符串向量
  std::vector<std::string> split(const std::string& s, char delimiter);
};

#endif  // DATABASE_H_