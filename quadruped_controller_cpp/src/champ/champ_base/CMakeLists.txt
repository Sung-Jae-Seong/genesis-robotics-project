cmake_minimum_required(VERSION 3.5)
project(champ_base)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(champ REQUIRED)


set(dependencies
  urdf
  champ
)

include_directories(
  include
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ/
  ${champ_INCLUDE_DIRS}/champ/champ/
)

add_executable(quadruped_controller src/quadruped_controller.cpp)
ament_target_dependencies(quadruped_controller ${dependencies})

install(TARGETS 
    quadruped_controller
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
