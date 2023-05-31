cmake_minimum_required(VERSION 3.8)
project(trailbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright # skip the copyright & license checker for now until it's added to all files
  )
  ament_lint_auto_find_test_dependencies()
endif()
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"srv/SnackWanted.srv"
"srv/RunServo.srv"
"msg/SnacksInventory.msg"
)
ament_package()
