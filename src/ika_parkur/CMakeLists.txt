cmake_minimum_required(VERSION 3.8)
project(kayar_engel_plugin)

# C++17 standardi
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Gazebo'yu bul
find_package(gazebo REQUIRED)

# Include dizinleri
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Plugin'i derle
add_library(kayar_engel_plugin SHARED kayar_engel_plugin.cc)
target_link_libraries(kayar_engel_plugin ${GAZEBO_LIBRARIES})

# Kurulum
install(TARGETS kayar_engel_plugin
  LIBRARY DESTINATION lib
)
