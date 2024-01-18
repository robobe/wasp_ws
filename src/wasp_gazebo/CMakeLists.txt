cmake_minimum_required(VERSION 3.8)
project(wasp_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
link_directories(${gazebo_dev_LIBRARY_DIRS})
add_library(test_plug SHARED src/test_plug.cc)
target_link_libraries(test_plug ${GAZEBO_LIBRARIES})

add_library(control_plugin
    SHARED 
    src/control_plugin.cpp
    src/fdcl_matrix_utils.cpp
    src/fdcl_ros_utils.cpp
)
ament_target_dependencies(control_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "geometry_msgs"
)
target_include_directories(control_plugin PUBLIC include)
ament_export_libraries(control_plugin)

install(TARGETS
test_plug
control_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY bin
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
