//
// Created by chiliu on 5/12/18.
//

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "common_include.h"

namespace myslam
{
    class Frame;
    class MapPoint
    {
    public:
        typedef shared_ptr<MapPoint> Ptr;
        unsigned long                id_;               // ID
        static unsigned long         factory_id_;       // factory ID
        bool                         good_;             // weather it is a good point
        Vector3d                     pos_;              // Position in the world
        Vector3d                     norm_;             // Normal of viewing direction
        Mat                          descriptor_;       // descriptor for matching

        list<Frame*>                 observed_frames_;  // key-frames that can observe this point
        int                          matched_times_;      // being an inliner in pose estimation
        int                          visible_times_;   // being visiable in current time

        MapPoint();
        MapPoint( unsigned long id, const Vector3d& position,
                  const Vector3d& norm, Frame* frame= nullptr,
                  const Mat& descriptor=Mat());

        inline cv::Point3f getPositionCV() const
        {
            return cv::Point3f( pos_(0, 0), pos_(1, 0), pos_(2, 0));
        }

        // factory function
        static MapPoint::Ptr createMapPoint();
        static MapPoint::Ptr createMapPoint(
                const Vector3d& pos_world,
                const Vector3d& norm,
                const Mat&descriptor,
                Frame* frame
        );

    };
}

#endif //MYSLAM_MAPPOINT_H
