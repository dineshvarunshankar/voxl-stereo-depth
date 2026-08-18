add_library(hetero_dfs_core STATIC
    config.cpp
    frame.cpp
    frame_pairer.cpp
)

target_include_directories(hetero_dfs_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

add_library(hetero_dfs_rectification STATIC
    calibration.cpp
    rectification_geometry.cpp
)

target_include_directories(hetero_dfs_rectification
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(hetero_dfs_rectification PUBLIC ${OpenCV_LIBS})

if(HETERO_DFS_BUILD_VOXL)
    find_library(MODAL_PIPE_LIBRARY modal_pipe REQUIRED HINTS /usr/lib64)
    add_executable(voxl-stereo-timing timing_probe.cpp)
    target_link_libraries(voxl-stereo-timing
        PRIVATE
            hetero_dfs_core
            ${MODAL_PIPE_LIBRARY}
            pthread
    )
    install(TARGETS voxl-stereo-timing RUNTIME DESTINATION /usr/bin)
endif()
