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
    depth_processor.cpp
    rectification_geometry.cpp
)

target_include_directories(hetero_dfs_rectification
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(hetero_dfs_rectification PUBLIC ${OpenCV_LIBS})

find_library(MODAL_PIPE_LIBRARY modal_pipe REQUIRED HINTS /usr/lib64)
find_library(MODAL_JSON_LIBRARY modal_json REQUIRED HINTS /usr/lib64)
find_path(MODAL_PIPE_INCLUDE_DIR modal_pipe.h REQUIRED)
find_path(MODAL_JSON_INCLUDE_DIR modal_json.h REQUIRED)

target_include_directories(hetero_dfs_core PUBLIC ${MODAL_JSON_INCLUDE_DIR})
target_include_directories(hetero_dfs_rectification PUBLIC ${MODAL_JSON_INCLUDE_DIR})
add_executable(voxl-stereo-timing timing_probe.cpp)
target_link_libraries(voxl-stereo-timing
    PRIVATE
        hetero_dfs_core
        ${MODAL_JSON_LIBRARY}
        ${MODAL_PIPE_LIBRARY}
        pthread
)
target_include_directories(voxl-stereo-timing PRIVATE ${MODAL_PIPE_INCLUDE_DIR})

add_executable(voxl-hetero-dfs-server stereo_service.cpp)
target_link_libraries(voxl-hetero-dfs-server
    PRIVATE
        hetero_dfs_core
        hetero_dfs_rectification
        ${MODAL_JSON_LIBRARY}
        ${MODAL_PIPE_LIBRARY}
        pthread
)
target_include_directories(voxl-hetero-dfs-server PRIVATE ${MODAL_PIPE_INCLUDE_DIR})

install(TARGETS
    voxl-stereo-timing
    voxl-hetero-dfs-server
    RUNTIME DESTINATION /usr/bin)
