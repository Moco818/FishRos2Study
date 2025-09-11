cmake_minimum_required(VERSION 3.8)
project(demo_cpp_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

# 1. 查找头文件和库
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(include)
# 2. 添加可执行文件
# add_executable(cpp_node src/cpp_node.cpp)
add_executable(turtle_circle src/turtle_circle.cpp)
# 3. 添加依赖
# ament_target_dependencies(cpp_node rclcpp)
ament_target_dependencies(turtle_circle rclcpp geometry_msgs)
# 4. 复制到 install 目录
install(TARGETS
  turtle_circle
  DESTINATION lib/${PROJECT_NAME}
)

# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
