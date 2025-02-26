cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(path_planning REQUIRED) 

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputePath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

include_directories(
  include
)

add_executable(path_planner src/planner_node.cpp src/a_star.cpp)
ament_target_dependencies(path_planner rclcpp nav_msgs visualization_msgs yaml-cpp path_planning)

add_executable(main src/main.cpp src/a_star.cpp)
ament_target_dependencies(main rclcpp nav_msgs visualization_msgs yaml-cpp)

add_executable(grid_map_node src/grid_map.cpp)
ament_target_dependencies(grid_map_node rclcpp nav_msgs visualization_msgs yaml-cpp)

add_executable(path_planner_service src/path_planner_service.cpp src/a_star.cpp)
ament_target_dependencies(path_planner_service rclcpp geometry_msgs nav_msgs path_planning)


target_link_libraries(grid_map_node yaml-cpp)
target_link_libraries(path_planner yaml-cpp)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS 
  path_planner
  path_planner_service
  main
  DESTINATION lib/${PROJECT_NAME}
  )

install(TARGETS grid_map_node
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
