cmake_minimum_required(VERSION 3.10.0)
project(RobotControl VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH "../bin")

find_package(PythonLibs)

include_directories(
    include
    ${PROJECT_SOURCE_DIR}/include
    /usr/local/include
)

link_directories(/usr/local/lib)

add_executable(PID ./src/PID_Control.cpp)
add_executable(LQR ./src/LQR_Control.cpp)
add_executable(MPC ./src/MPC_Control.cpp)

target_include_directories(PID PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include" ${PYTHON_INCLUDE_DIRS}) #指定头文件搜索文件夹
target_link_libraries(PID ${PYTHON_LIBRARIES})

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
