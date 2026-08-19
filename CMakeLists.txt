cmake_minimum_required(VERSION 3.10)

project(voxl-hetero-dfs-server LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

add_compile_options(-Wall -Wextra -Werror -Wuninitialized -Werror=undef)

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)

add_subdirectory(src)

install(FILES config/voxl-hetero-dfs-server.conf DESTINATION /etc/modalai)
install(FILES services/voxl-hetero-dfs-server.service DESTINATION /etc/systemd/system)
install(FILES
    calibration/opencv_tracking_front_intrinsics.yml
    calibration/opencv_hires_small_intrinsics.yml
    DESTINATION /usr/share/modalai/voxl-hetero-dfs-server
)

