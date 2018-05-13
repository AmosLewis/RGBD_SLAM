add_library( myslam SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
        Camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)