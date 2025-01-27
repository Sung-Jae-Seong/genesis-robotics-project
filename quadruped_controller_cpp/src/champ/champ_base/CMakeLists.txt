cmake_minimum_required(VERSION 3.5)
project(champ_base)

add_compile_options(-std=c++17)

set(CMAKE_PREFIX_PATH "/opt/ros/${ROS_DISTRO};/usr/local/lib;/usr/local/include")

find_package(urdf REQUIRED)
find_package(champ REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

include_directories(
  include
  ${urdf_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  /usr/local/include/champ #champ path
)
link_libraries(${Python3_LIBRARIES})

add_executable(quadruped_controller src/quadruped_controller.cpp)
ament_target_dependencies(quadruped_controller urdf champ)

target_link_libraries(quadruped_controller ${urdf_LIBRARIES} champ_lib)

install(TARGETS 
  quadruped_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pybind11_add_module(quadruped_controller_binding src/quadruped_controller)
target_link_libraries(quadruped_controller_binding PRIVATE ${urdf_LIBRARIES} champ_lib)