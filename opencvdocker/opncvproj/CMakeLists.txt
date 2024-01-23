cmake_minimum_required(VERSION 3.10)

project(MyProject)

# set(OpenCV_DIR /opencv_build)
find_package(OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} src/serial.cpp)
# add_executable(${PROJECT_NAME} src/yolo.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})