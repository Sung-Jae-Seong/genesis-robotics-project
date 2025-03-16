cmake_minimum_required(VERSION 3.5)
project(champ_base)

add_compile_options(-std=c++17)

find_package(urdf REQUIRED)
find_package(champ REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(URDF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/resource/robot.urdf")
set(GAIT_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/resource/gait_config.yaml")
set(JOINTS_MAP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/resource/joints_map.yaml")
set(LINKS_MAP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/resource/links_map.yaml")

configure_file(${URDF_FILE} ${CMAKE_CURRENT_BINARY_DIR}/resource/robot_urdf.h COPYONLY)
configure_file(${GAIT_CONFIG_FILE} ${CMAKE_CURRENT_BINARY_DIR}/resource/gait_config.h COPYONLY)
configure_file(${JOINTS_MAP_FILE} ${CMAKE_CURRENT_BINARY_DIR}/resource/joints_map.h COPYONLY)
configure_file(${LINKS_MAP_FILE} ${CMAKE_CURRENT_BINARY_DIR}/resource/links_map.h COPYONLY)

pybind11_add_module(quadruped_controller 
  src/quadruped_controller.cpp
  src/yaml_parser.cpp
)

set_target_properties(quadruped_controller PROPERTIES 
  OUTPUT_NAME "quadruped_controller"
  SUFFIX ".so"
)

target_include_directories(quadruped_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${urdf_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  /usr/local/include/champ
)

target_link_libraries(quadruped_controller PRIVATE
  champ_lib
  ${urdf_LIBRARIES}
  ${Python3_LIBRARIES}
  yaml-cpp
)

install(TARGETS 
  quadruped_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
