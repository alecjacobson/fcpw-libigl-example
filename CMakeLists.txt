cmake_minimum_required(VERSION 3.1.0)
project(fcpw-libigl-example)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(libigl)
add_subdirectory(fcpw)
add_executable(${PROJECT_NAME} main.cpp )

target_link_libraries(${PROJECT_NAME} fcpw igl::core)
target_include_directories(${PROJECT_NAME} PUBLIC ${FCPW_EIGEN_INCLUDES})
target_include_directories(${PROJECT_NAME} PUBLIC ${FCPW_ENOKI_INCLUDES})
