add_library( myslam SHARED
        Frame.cpp
    mappoint.cpp
    map.cpp
        Camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp
        ../test/run_vo.cpp)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)