cmake_minimum_required(VERSION 3.10)
project(SenseGloveClient LANGUAGES CXX)

# 强制使用 Clang 和 libc++
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_CXX_FLAGS "-stdlib=libc++ -fPIC")

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件和库路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/v22/x86-64/debug)

# 查找库
find_library(SGCORE_LIB NAMES sgcore PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/v22/x86-64/debug NO_DEFAULT_PATH)
find_library(SGCONNECT_LIB NAMES sgconnect PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/v22/x86-64/debug NO_DEFAULT_PATH)

if(NOT SGCORE_LIB OR NOT SGCONNECT_LIB)
    message(FATAL_ERROR "SenseGlove libraries not found!")
endif()

# 可执行文件
add_executable(client ${CMAKE_CURRENT_SOURCE_DIR}/examples/sgcore-client.cpp)

# 链接时也指定 -stdlib=libc++
target_link_libraries(client 
    ${SGCORE_LIB} 
    ${SGCONNECT_LIB}
    -lc++ 
    -lc++abi
)

# 输出到 build/
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})