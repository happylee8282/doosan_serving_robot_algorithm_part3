cmake_minimum_required(VERSION 3.8)
project(food_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rosidl_default_generators REQUIRED)

#의존성 명시: std_msgs 또는 추가로 필요한 메시지/서비스 패키지가 있다면 find_package()로 추가해야 합니다.
find_package(std_msgs REQUIRED)

set(msg_files
 "msg/Ordermsg.msg"
 "msg/Waypoint.msg"
)

set(srv_files
 "srv/Foodcustomer.srv"
 "srv/Datawait.srv"
 "srv/Navigationok.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
${msg_files}
${srv_files}
DEPENDENCIES builtin_interfaces
)

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
