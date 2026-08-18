cmake_minimum_required(VERSION 3.10)

project(voxl-hetero-dfs-server LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

add_compile_options(-Wall -Wextra -Werror -Wuninitialized -Werror=undef)

option(HETERO_DFS_BUILD_VOXL "Build binaries that require VOXL SDK libraries" OFF)

find_package(OpenCV REQUIRED COMPONENTS core)

add_subdirectory(src)

