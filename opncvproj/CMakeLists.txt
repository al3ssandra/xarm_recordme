cmake_minimum_required(VERSION 3.10)

project(MyProject)

include_directories( ${OpenCV_INCLUDE_DIRS} 
                    /orocos_kinematics_dynamics/orocos_kdl/src
                    ${PROJECT_SOURCE_DIR}/src
                    )

# set(OpenCV_DIR /opencv_build)
find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)

# add_executable(${PROJECT_NAME} src/trajectory.cpp)
# add_executable(${PROJECT_NAME} src/ros_kin.cpp)
add_executable(${PROJECT_NAME} src/control.cpp)
# add_executable(${PROJECT_NAME} src/yolo.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} orocos-kdl)