add_library( myslam SHARED
        Frame.cpp
        MapPoint.cpp
        Map.cpp
        Camera.cpp
        Config.cpp
    g2o_types.cpp
        VisualOdometry.cpp
        ../test/run_vo.cpp)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)