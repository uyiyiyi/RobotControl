cmake_minimum_required(VERSION 3.10.0)
project(RobotControl VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH "../bin")


add_executable(PID ./src/PID_Control.cpp)
add_executable(LQR ./src/LQR_Control.cpp)
add_executable(MPC ./src/MPC_Control.cpp)

target_include_directories(PID PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include") #指定头文件搜索文件夹

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
