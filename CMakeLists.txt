cmake_minimum_required(VERSION 3.22.1)
project(wheel_leg_sys)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_Release "-O3 -march=native -DNDEBUG")

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# =========================================================
# 依赖查找 (Dependencies)
# =========================================================
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CPPAD REQUIRED cppad)
find_package(PkgConfig)
pkg_search_module(LIBUSB REQUIRED libusb-1.0)

# 尝试自动查找 CasADi，找不到再用硬编码
find_package(casadi CONFIG)
if(NOT casadi_FOUND)
    message(STATUS "CasADi config not found, trying to find library manually...")
    find_library(CASADI_LIB casadi PATHS /usr/local/lib /usr/lib)
    if(NOT CASADI_LIB)
        # 如果还没找到，为了兼容你的环境，回退到你原来的硬编码路径，但建议检查安装
        set(CASADI_LIB "/usr/local/lib/libcasadi.so.3.7")
        message(WARNING "Using hardcoded CasADi path: ${CASADI_LIB}")
    endif()
endif()

# =========================================================
# 头文件包含 (Include Directories)
# =========================================================
include_directories(
        src
        src/Algorithm/EKF
        src/Algorithm/KalmanFilter
        src/Algorithm/MPC
        src/Algorithm/PID
        src/Task
        src/Bsp
        src/Devices
        src/Algorithm/CRC
        ${EIGEN3_INCLUDE_DIRS}
        ${CPPAD_INCLUDE_DIRS}
)

# =========================================================
# 源文件管理 (Source Files)
# =========================================================
file(GLOB_RECURSE SOURCES
        "src/*.cpp"
        "src/*.c"
        "src/*.h"
        "src/*.hpp"
)

# =========================================================
# 构建目标 (Target)
# =========================================================
add_executable(wheel_leg_sys ${SOURCES})

# =========================================================
# 链接库 (Linking)
# =========================================================
target_link_libraries(wheel_leg_sys PRIVATE qpOASES)
target_compile_options(wheel_leg_sys PRIVATE ${CPPAD_CFLAGS_OTHER})

# 链接 CasADi
if(casadi_FOUND)
    target_link_libraries(wheel_leg_sys PRIVATE casadi)
else()
    target_link_libraries(wheel_leg_sys PRIVATE ${CASADI_LIB})
endif()

target_include_directories(${PROJECT_NAME} PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBUSB_LIBRARIES} pthread)