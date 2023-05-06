cmake_minimum_required(VERSION 3.8)
project(flir_a320_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_library(node SHARED src/main.cpp)
ament_target_dependencies(node ament_index_cpp rclcpp rclcpp_components cv_bridge image_transport OpenCV)
target_include_directories(node PUBLIC /opt/spinnaker/include)
target_link_libraries(node /opt/spinnaker/lib/libSpinnaker.so)
rclcpp_components_register_node(node
PLUGIN "flir_vision_ros2::FlirVision"
EXECUTABLE flir_node)

install(TARGETS
  node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

ament_package()
