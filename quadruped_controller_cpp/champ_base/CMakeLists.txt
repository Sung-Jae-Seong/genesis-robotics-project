cmake_minimum_required(VERSION 3.5)
project(champ_base)

add_compile_options(-std=c++17)

find_package(urdf REQUIRED)
find_package(champ REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(yaml-cpp REQUIRED)

pybind11_add_module(quadruped_controller_binding 
  src/quadruped_controller.cpp
  src/yaml_parser.cpp
)

target_include_directories(quadruped_controller_binding PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${urdf_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  /usr/local/include/champ
)

target_link_libraries(quadruped_controller_binding PRIVATE
  champ_lib
  ${urdf_LIBRARIES}
  ${Python3_LIBRARIES}
  yaml-cpp
)

install(TARGETS 
  quadruped_controller_binding
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
