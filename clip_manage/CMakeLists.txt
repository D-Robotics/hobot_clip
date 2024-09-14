cmake_minimum_required(VERSION 3.5)
project(clip_manage)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_BUILD_TYPE RelWithDebInfo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(clip_msgs REQUIRED)

include_directories(include
  ${PROJECT_SOURCE_DIR}
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/clip_manage_node.cpp
  src/database.cpp
  src/feature_workflow.cpp
  src/item.cpp
  src/image_action_client.cpp
  src/text_action_client.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_action
  SQLite3
  clip_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${PROJECT_SOURCE_DIR}/config/index.html
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
