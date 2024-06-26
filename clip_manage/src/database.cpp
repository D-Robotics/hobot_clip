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
#include <sstream>
#include <algorithm>
#include <iterator>
#include <cstring>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "include/database.h"

void ClipItemDatabase::initialize(const std::string& db_name) {
  if (db) {
    sqlite3_close(db);
    db = nullptr;
  }
  this->db_name = db_name;
  openDatabase(db_name);
}

void ClipItemDatabase::openDatabase(const std::string& db_name) {
  if (sqlite3_open(db_name.c_str(), &db)) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
                  "Can't open database: %s", sqlite3_errmsg(db));
    sqlite3_close(db);
    db = nullptr;
  }
}

ClipItemDatabase::~ClipItemDatabase() {
  if (db) {
    sqlite3_close(db);
  }
}

bool ClipItemDatabase::createTable() {
  const char* sql = "CREATE TABLE IF NOT EXISTS ClipItems ("
                "id INTEGER PRIMARY KEY AUTOINCREMENT,"
                "timestamp INTEGER,"
                "type BOOLEAN,"
                "name TEXT,"
                "text TEXT,"
                "url TEXT,"
                "feature BLOB,"
                "extra TEXT);";
  return executeSQL(sql);
}

bool ClipItemDatabase::insertItem(const ClipItem& item) {
  // Check if item is valid
  if (!isValidItem(item)) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
        "Invalid item. Item must have non-empty URL and feature.");
    return false;
  }

  // Check if item with the same URL already exists
  if (itemExists(item)) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
        "Item with URL %s already exists. Skipping insertion.", item.url.c_str());
    return true; // Assuming item already exists is not an error condition
  }

  // Get current system timestamp
  long current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  const char* sql = "INSERT INTO ClipItems (id, timestamp, type, name, text, url, feature, extra) "
                    "VALUES (NULL, ?, ?, ?, ?, ?, ?, ?);"; // id 字段设为 NULL，由数据库自动生成
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
            "Failed to prepare statement:  %s", sqlite3_errmsg(db));
    return false;
  }
  sqlite3_bind_int64(stmt, 1, current_timestamp);
  sqlite3_bind_int(stmt, 2, item.type);
  sqlite3_bind_text(stmt, 3, item.name.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 4, item.text.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 5, item.url.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_blob(stmt, 6, item.feature.data(), item.feature.size() * sizeof(float), SQLITE_STATIC);
  std::string extra = join(item.extra, ",");
  sqlite3_bind_text(stmt, 7, extra.c_str(), -1, SQLITE_STATIC);

  if (sqlite3_step(stmt) != SQLITE_DONE) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
            "Failed to execute statement:  %s", sqlite3_errmsg(db));
    sqlite3_finalize(stmt);
    return false;
  }

  // Get the last inserted row ID
  int last_id = sqlite3_last_insert_rowid(db);
  if (last_id <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
            "Failed to get last inserted row ID.");
    sqlite3_finalize(stmt);
    return false;
  }

  // Update the item's id with the last inserted row ID
  ClipItem updated_item = item;
  updated_item.id = last_id;

  sqlite3_finalize(stmt);
  return true;
}


bool ClipItemDatabase::itemExists(const ClipItem& item) {
  return urlExists(item.url);
}

bool ClipItemDatabase::urlExists(const std::string& url) {
  const char* sql = "SELECT COUNT(*) FROM ClipItems WHERE url = ?;";
  sqlite3_stmt* stmt;
  int result = 0;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
            "Failed to prepare statement:  %s", sqlite3_errmsg(db));
    return false;
  }
  sqlite3_bind_text(stmt, 1, url.c_str(), -1, SQLITE_STATIC);
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    result = sqlite3_column_int(stmt, 0);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
            "Failed to execute statement:  %s", sqlite3_errmsg(db));    
    sqlite3_finalize(stmt);
    return false;
  }
  sqlite3_finalize(stmt);
  return (result > 0);
}

bool ClipItemDatabase::isValidItem(const ClipItem& item) {
  return (!item.url.empty() && !item.feature.empty());
}

std::string ClipItemDatabase::join(const std::vector<std::string>& vec, const std::string& delimiter) {
  std::ostringstream oss;
  if (!vec.empty()) {
    std::copy(vec.begin(), vec.end() - 1, std::ostream_iterator<std::string>(oss, delimiter.c_str()));
    oss << vec.back();
  }
  return oss.str();
}

std::vector<std::string> ClipItemDatabase::split(const std::string& s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}


bool ClipItemDatabase::deleteItem(int id) {
  const char* sql = "DELETE FROM ClipItems WHERE id = ?;";
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
        "Delete failed. Failed to prepare statement: %s", sqlite3_errmsg(db));
    return false;
  }
  sqlite3_bind_int(stmt, 1, id);
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
        "Delete failed. Failed to execute statement: %s", sqlite3_errmsg(db));
    sqlite3_finalize(stmt);
    return false;
  }
  sqlite3_finalize(stmt);
  return true;
}

ClipItem ClipItemDatabase::queryItem(int id) {
  const char* sql = "SELECT timestamp, type, name, text, url, feature, extra FROM ClipItems WHERE id = ?;";
  sqlite3_stmt* stmt;
  ClipItem item;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
        "Query failed. Failed to prepare statement: %s", sqlite3_errmsg(db));
    return item;
  }
  sqlite3_bind_int(stmt, 1, id);
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    item.id = id;
    item.timestamp = sqlite3_column_int64(stmt, 0);
    item.type = sqlite3_column_int(stmt, 1);
    item.name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
    item.text = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
    item.url = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));

    // Retrieve feature data
    const void* feature_blob = sqlite3_column_blob(stmt, 5);
    int feature_size = sqlite3_column_bytes(stmt, 5);
    if (feature_blob && feature_size > 0) {
        item.feature.resize(feature_size / sizeof(float));
        memcpy(item.feature.data(), feature_blob, feature_size);
    }

    // Retrieve extra data
    const char* extra_blob = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6));
    if (extra_blob) {
        item.extra = split(extra_blob, ',');
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
        "No item found with id: %d", id);
  }
  sqlite3_finalize(stmt);
  return item;
}


bool ClipItemDatabase::executeSQL(const char* sql) {
    char* errMsg = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &errMsg) != SQLITE_OK) {
        RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
            "SQL error: %s", errMsg);
        sqlite3_free(errMsg);
        return false;
    }
    return true;
}

std::vector<ClipItem> ClipItemDatabase::queryItemsByPage(int page_number, int page_size) {
    const char* sql = "SELECT id, timestamp, type, name, text, url, feature, extra FROM ClipItems LIMIT ? OFFSET ?;";
    sqlite3_stmt* stmt;
    std::vector<ClipItem> items;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
            "Query page failed. Failed to prepare statement: %s", sqlite3_errmsg(db));
        return items;
    }
    int offset = (page_number - 1) * page_size;
    sqlite3_bind_int(stmt, 1, page_size);
    sqlite3_bind_int(stmt, 2, offset);
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        ClipItem item;
        item.id = sqlite3_column_int(stmt, 0);
        item.timestamp = sqlite3_column_int64(stmt, 1);
        item.type = sqlite3_column_int(stmt, 2);
        item.name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
        item.text = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
        item.url = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5));

        // Retrieve feature data
        const void* feature_blob = sqlite3_column_blob(stmt, 6);
        int feature_size = sqlite3_column_bytes(stmt, 6);
        if (feature_blob && feature_size > 0) {
            item.feature.resize(feature_size / sizeof(float));
            memcpy(item.feature.data(), feature_blob, feature_size);
        }

        // Retrieve extra data
        const char* extra_blob = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 7));
        if (extra_blob) {
            std::string extra_str(extra_blob);
            // Assuming extra are separated by commas, adjust the delimiter as needed
            size_t pos = 0;
            while ((pos = extra_str.find(',')) != std::string::npos) {
                item.extra.push_back(extra_str.substr(0, pos));
                extra_str.erase(0, pos + 1);
            }
            if (!extra_str.empty()) {
                item.extra.push_back(extra_str);
            }
        }

        items.push_back(item);
    }
    sqlite3_finalize(stmt);
    return items;
}

int ClipItemDatabase::getItemCount() {
    const char* sql = "SELECT COUNT(*) FROM ClipItems;";
    sqlite3_stmt* stmt;
    int count = 0;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(rclcpp::get_logger("ClipNode"),
            "Get item count failed. Failed to prepare statement: %s", sqlite3_errmsg(db));
        return count;
    }
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        count = sqlite3_column_int(stmt, 0);
    }
    sqlite3_finalize(stmt);
    return count;
}
