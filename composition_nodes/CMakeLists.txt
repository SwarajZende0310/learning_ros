cmake_minimum_required(VERSION 3.8)
project(composition_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# # create ament index resource which references the libraries in the binary dir
# set(node_plugins "")

# add_library(talker_component SHARED
#   src/talker_component.cpp)
# target_compile_definitions(talker_component
#   PRIVATE "COMPOSITION_BUILDING_DLL")
# ament_target_dependencies(talker_component
#   "rclcpp"
#   "rclcpp_components"
#   "std_msgs")
# rclcpp_components_register_nodes(talker_component "composition::Talker")
# set(node_plugins "${node_plugins}composition::Talker;$<TARGET_FILE:talker_component>\n")

# add_library(listener_component SHARED
#   src/listener_component.cpp)
# target_compile_definitions(listener_component
#   PRIVATE "COMPOSITION_BUILDING_DLL")
# ament_target_dependencies(listener_component
#   "rclcpp"
#   "rclcpp_components"
#   "std_msgs")
# rclcpp_components_register_nodes(listener_component "composition::Listener")
# set(node_plugins "${node_plugins}composition::Listener;$<TARGET_FILE:listener_component>\n")

add_library(talker_component src/talker_component.cpp)
ament_target_dependencies(talker_component rclcpp std_msgs rclcpp_components)

add_library(listener_component src/listener_component.cpp)
ament_target_dependencies(listener_component rclcpp std_msgs rclcpp_components)

install(TARGETS
talker_component
listener_component
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)

ament_package()
