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

function(embed_file_as_header INPUT_FILE OUTPUT_HEADER VARIABLE_NAME)
  file(READ "${INPUT_FILE}" FILE_CONTENT)
  string(REPLACE "\\" "\\\\" FILE_CONTENT "${FILE_CONTENT}")
  string(REPLACE "\"" "\\\"" FILE_CONTENT "${FILE_CONTENT}")
  string(REPLACE "\n" "\\n\"\n\"" FILE_CONTENT "${FILE_CONTENT}")
  file(WRITE "${OUTPUT_HEADER}" "const char* ${VARIABLE_NAME} = \"${FILE_CONTENT}\";\n")
endfunction()

embed_file_as_header(${URDF_FILE} ${CMAKE_CURRENT_BINARY_DIR}/default_robot_urdf.h default_urdf_string)
embed_file_as_header(${GAIT_CONFIG_FILE} ${CMAKE_CURRENT_BINARY_DIR}/default_gait_config.h default_gait_yaml)
embed_file_as_header(${JOINTS_MAP_FILE} ${CMAKE_CURRENT_BINARY_DIR}/default_joints_map.h default_joints_yaml)
embed_file_as_header(${LINKS_MAP_FILE} ${CMAKE_CURRENT_BINARY_DIR}/default_links_map.h default_links_yaml)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

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
  ${CMAKE_CURRENT_BINARY_DIR}
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
