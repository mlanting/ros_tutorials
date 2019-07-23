cmake_minimum_required(VERSION 3.5)
project(turtlesim)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include
                    ${rclcpp_INCLUDE_DIRS}
                    ${rmw_implementation_INCLUDE_DIRS}
                    ${ament_index_cpp_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS}
                    ${std_srvs_INCLUDE_DIRS}
                    ${geometry_msgs_INCLUDE_DIRS}
                    ${Qt5Widgets_INCLUDE_DIRS}
                    )

set(msg_files
    msg/Color.msg
    msg/Pose.msg
)
set(srv_files
    srv/Kill.srv
    srv/SetPen.srv
    srv/Spawn.srv
    srv/TeleportAbsolute.srv
    srv/TeleportRelative.srv
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs std_srvs
)


set(turtlesim_node_SRCS
  src/turtlesim.cpp
  src/turtle.cpp
  src/turtle_frame.cpp
)
set(turtlesim_node_HDRS
  include/turtlesim/turtle_frame.hpp
  include/turtlesim/turtle.hpp
)

qt5_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})


# Turtlesim Node
add_executable(turtlesim_node ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
target_link_libraries(turtlesim_node Qt5::Widgets
                                     ${rclcpp_LIBRARIES}
                                     ${rmw_implementation_LIBRARIES}
                                     ${ament_index_cpp_LIBRARIES}
                                     ${std_msgs_LIBRARIES}
                                     ${std_srvs_LIBRARIES}
                                     ${geometry_msgs_LIBRARIES}
                                     )
rosidl_target_interfaces(turtlesim_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Turtle Teleop Key
add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
target_link_libraries(turtle_teleop_key ${rclcpp_LIBRARIES}
                                        ${rmw_implementation_LIBRARIES}
                                        ${ament_index_cpp_LIBRARIES}
                                        ${std_msgs_LIBRARIES}
                                        ${std_srvs_LIBRARIES}
                                        ${geometry_msgs_LIBRARIES}
                                        )
rosidl_target_interfaces(turtle_teleop_key ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Draw Square
add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square ${rclcpp_LIBRARIES}
                                  ${rmw_implementation_LIBRARIES}
                                  ${ament_index_cpp_LIBRARIES}
                                  ${std_msgs_LIBRARIES}
                                  ${std_srvs_LIBRARIES}
                                  ${geometry_msgs_LIBRARIES}
                                  )
rosidl_target_interfaces(draw_square ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Mimic
add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic ${rclcpp_LIBRARIES}
                            ${rmw_implementation_LIBRARIES}
                            ${ament_index_cpp_LIBRARIES}
                            ${std_msgs_LIBRARIES}
                            ${std_srvs_LIBRARIES}
                            ${geometry_msgs_LIBRARIES}
                            )
rosidl_target_interfaces(mimic ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(geometry_msgs rosidl_default_runtime std_msgs std_srvs)
ament_package()

install(TARGETS turtlesim_node turtle_teleop_key draw_square mimic
    RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY images
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.py" PATTERN "*.launch")

