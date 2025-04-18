cmake_minimum_required(VERSION 3.5)
project(champ)

set(LIBRARY_NAME "${PROJECT_NAME}_lib")

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${LIBRARY_NAME}
  EXPORT ${PROJECT_NAME}Targets
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

export(TARGETS ${LIBRARY_NAME} FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake)
