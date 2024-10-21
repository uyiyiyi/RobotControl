cmake_minimum_required(VERSION 3.10.0)
project(RobotControl VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(EXECUTABLE_OUTPUT_PATH "../bin")

# 查找 yaml-cpp 库
find_package(yaml-cpp REQUIRED)
find_package(PythonLibs 3.8)

include_directories(
    include
    ${PROJECT_SOURCE_DIR}/include
    /usr/local/include
)

link_directories(/usr/local/lib)

add_executable(PID ./src/PID_Control.cpp)
# add_executable(LQR ./src/LQR_Control.cpp)
# add_executable(MPC ./src/MPC_Control.cpp)

target_include_directories(PID PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include" ${PYTHON_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include include) #指定头文件搜索文件夹
target_link_libraries(PID ${PYTHON_LIBRARIES} yaml-cpp)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
