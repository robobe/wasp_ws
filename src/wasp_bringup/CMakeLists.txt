cmake_minimum_required(VERSION 3.8)
project(wasp_bringup)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${GAZEBO_INCLUDE_DIRS}
        )

link_libraries(
        ${GAZEBO_LIBRARIES}
        )

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        )

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_library(GimbalSmall2dPlugin_new SHARED src/GimbalSmall2dPlugin.cc)
target_link_libraries(GimbalSmall2dPlugin_new ${GAZEBO_LIBRARIES})

install(DIRECTORY
  launch
  worlds
  bin
  models
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  GimbalSmall2dPlugin_new
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
