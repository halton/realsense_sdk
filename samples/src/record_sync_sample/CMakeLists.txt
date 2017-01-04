cmake_minimum_required(VERSION 2.8)
project(realsense_record_sync_sample)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(${PROJECT_NAME} main.cpp)

find_package(PkgConfig)

pkg_check_modules(JSONCPP QUIET jsoncpp)

include_directories(${JSONCPP_INCLUDE_DIRS})

link_directories(${LINK_DIRECTORIES} ${JSONCPP_LIBDIR})

target_link_libraries(${PROJECT_NAME} realsense_image
                                      realsense_record
                                      realsense_log_utils
                                      realsense pthread
                                      opencv_imgproc
                                      opencv_core
                                      ${JSONCPP_LIBRARIES})


install(TARGETS ${PROJECT_NAME} DESTINATION bin)
