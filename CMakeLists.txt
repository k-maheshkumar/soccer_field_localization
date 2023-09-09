cmake_minimum_required(VERSION 3.22)
project(soccer_field_localization VERSION 0.1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-pie")

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

find_library(
  controller
  NAMES controller
  HINTS "${CMAKE_SOURCE_DIR}/lib")

add_executable(localization_test src/main.cpp)
target_include_directories(localization_test PRIVATE include)
target_link_libraries(localization_test PRIVATE ${controller} GLUT::GLUT
                                                OpenGL::OpenGL)
