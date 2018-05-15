//
// Created by chiliu on 5/12/18.
//

#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H
#include "common_include.h"
#include "Map.h"
#include "g2o_types.h"
#include "Config.h"


namespace myslam
{
    class VisualOdometry
    {
    public:
        typedef shared_ptr<VisualOdometry> Ptr;
        enum VOState {
            INITIALIZING=-1,
            OK=0,
            LOST
        };

        VOState         state_;                 // current VO status
        Map::Ptr        map_;                   // map with all frames and points

        Frame::Ptr      ref_;                   // referrence key-frame
        Frame::Ptr      curr_;                  // current frame

        cv::Ptr<cv::ORB>        orb_;              // orb detector and computer
        vector<cv::KeyPoint>    keypoints_curr_;   // keypoints in current frame, output of orb_->detect
        Mat                     descriptors_curr_; // descriptor in current frame, output of orb_->detect
        Mat                     descriptors_ref_;  // descriptor in current frame, output of orb_->detect
        vector<cv::DMatch>      feature_matches_;  // all good matches
        vector<cv::Point3f>     pts_3d_ref_;       // 3d points in reference frame

        cv::FlannBasedMatcher   matcher_flann_;    // flann matcher
        vector<MapPoint::Ptr>   match_3dpts_;      // matched 3D points
        vector<int>             match_2dkp_index_; // matched 2D pixels (index of keypoints_curr)

        SE3 T_c_w_estimated_;     // the estimated pose of current frame
        int num_inliers_;         // number of inlier features in pnp_ransac
        int num_lost_;            // number of lost times

        // parameters
        int num_of_features_;     // number of features
        double scale_factor_;     // scale of image pyramid
        int level_pyramid_;       // number of pyramid levels
        float match_ratio_;       // ratio for selecting good matches
        int max_num_lost_;        // max number of continuous lost times
        int min_inliers_;         // minimum inliers

        double key_frame_min_rot;       // minimal rotation of two key frames
        double key_frame_min_trans;     // minimal translation of two frames

        double map_point_erase_ratio_;  // remove map point ratio

    public:
        VisualOdometry();
        ~VisualOdometry();

        bool addFrame( Frame::Ptr frame);           // add a new frame

    protected:
        // inner operation
        void extractKeyPoints();
        void computeDescriptors();
        void featureMatching();
        void poseEstimationPnP();
        void setRef3DPoints();
        void optimizeMap();

        void addKeyFrame();
        void addMapPoints();
        bool checkEstimatePose();
        bool checkKeyFrame();

        double getViewAngle( Frame::Ptr, MapPoint::Ptr point);

    };
}

#endif //MYSLAM_VISUAL_ODOMETRY_H











