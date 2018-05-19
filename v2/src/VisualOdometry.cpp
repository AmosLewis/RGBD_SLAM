//
// Created by chiliu on 5/12/18.
//

#include "../include/myslam/VisualOdometry.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "../include/myslam/g2o_types.h"

namespace myslam
{
    VisualOdometry::VisualOdometry():
        state_(INITIALIZING), ref_(nullptr), curr_(nullptr), map_(new Map), num_lost_(0), num_inliers_(0)
            //, matcher_flann_( new cv::flann::LshIndexParams (5,10,2))
    {
        num_of_features_        = Config::get<int> ("number_of_features");
        scale_factor_           = Config::get<double> ("scale_factor");
        level_pyramid_          = Config::get<int> ("level_pyramid");
        match_ratio_            = Config::get<float> ("match_ratio");
        max_num_lost_           = Config::get<float> ("max_num_lost");
        min_inliers_            = Config::get<int> ("min_inliers");
        key_frame_min_rot       = Config::get<double> ("keyframe_rotation");
        key_frame_min_trans     = Config::get<double> ("keyframe_translation");
        map_point_erase_ratio_  = Config::get<double> ("map_point_erase_ratio");
        orb_ = cv::ORB::create( num_of_features_, scale_factor_, level_pyramid_ );
    }

    VisualOdometry::~VisualOdometry()
    {

    }

    bool VisualOdometry::addFrame(Frame::Ptr frame)
    {
        switch ( state_ )
        {
            case INITIALIZING:
            {
                state_ =OK;
                curr_ = ref_ = frame;
                map_->insertKeyFrame( frame );
                // extract features from first frame
                extractKeyPoints();
                computeDescriptors();
                // compute the 3d position of features in ref frame
                setRef3DPoints();
                break;
            }
            case OK:
            {
                curr_ = frame;
                extractKeyPoints();                 // orb->detect
                computeDescriptors();               // orb->compute
                featureMatching();                  // calculate good matches
                poseEstimationPnP();                // calculate T_c_r_estimated_
                if (checkEstimatePose() == true )   // this is a good estimation
                {
                    // T_c_w = T_c_r * T_r_w
                    curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;
                    ref_ = curr_;
                    setRef3DPoints();
                    num_lost_ = 0;
                    if ( checkKeyFrame() == true )  // this is a key frame
                    {
                        addKeyFrame();
                    }
                }
                else    // bad estimation due to various reasons
                {
                    num_lost_++;
                    if ( num_lost_ > max_num_lost_ )
                    {
                        state_ = LOST;
                    }
                    return false;
                }
                break;
            }
            case LOST:
            {
                cout<<" vo has lost."<<endl;
                break;
            }
        }
        return true;
    }

    void VisualOdometry::extractKeyPoints()
    {
        orb_->detect ( curr_->color_, keypoints_curr_);
    }

    void VisualOdometry::computeDescriptors()
    {
        orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_);
    }

    void VisualOdometry::featureMatching()
    {
        // match desp_ref and desp_curr, use OpencCV's burte force match
        vector<cv::DMatch> matches;
        cv::BFMatcher matcher (cv::NORM_HAMMING);
        matcher.match( descriptors_ref_, descriptors_curr_, matches );

        // select good matches
        //// find minimum distance in all matches
        float min_dis = std::min_element(
                matches.begin(), matches.end(),
                []( const cv::DMatch& m1, const cv::DMatch& m2 )
                {
                    return m1.distance < m2.distance;
                }
        )->distance;
        ////  store good matches
        feature_matches_.clear();
        for( cv::DMatch& m: matches )
        {
            if ( m.distance < max<float>( min_dis * match_ratio_, 30.0 ) )
            {
                feature_matches_.push_back(m);
            }
        }
        cout<< " good matches: "<<feature_matches_.size()<<endl;


    }

    void VisualOdometry::setRef3DPoints()
    {
        // select the features with depth measurements
        pts_3d_ref_.clear();
        descriptors_ref_ = Mat();
        for ( size_t i=0; i<keypoints_curr_.size(); i++)
        {
            // calculate depth of every keypoint in current frame
            double d = ref_->findDepth(keypoints_curr_[i]);
            if ( d > 0)
            {
                // 2D-->3D use eigen to calculate camera coordinate points
                Vector3d p_cam = ref_->camera_->pixel2camera(
                        Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),d
                );
                // eigen --> opencv, save opencv 3D point
                pts_3d_ref_.push_back( cv::Point3f(p_cam(0,0), p_cam(1,0), p_cam(2,0)));
                descriptors_ref_.push_back( descriptors_curr_.row(i));
            }
        }

    }

    void VisualOdometry::poseEstimationPnP()
    {
        // construct 3d->2d obersvations
        //// prepare 3d and 2d points
        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;

        for (cv::DMatch m: feature_matches_)
        {
            pts3d.push_back(pts_3d_ref_[m.queryIdx]);
            pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
        }
        cout<<"last pts_3d_ref_[m.queryIdx]: "<<pts3d[pts3d.size()-1]<<endl;
        cout<<"last keypoints_curr_[m.trainIdx].pt: "<<pts2d[pts2d.size()-1]<<endl;
        cout<<"pts3d.size(): "<<pts3d.size()<<endl;
        cout<<"pts2d.size(): "<<pts2d.size()<<endl;

        //// prepare intrinsic matrix of camera
        Mat K = (cv::Mat_<double>(3,3)<<
            ref_->camera_->fx_, 0, ref_->camera_->cx_,
            0, ref_->camera_->fy_, ref_->camera_->cy_,
            0,0,1
        );

//        cout<<" K : "<<endl;
//        cout<<ref_->camera_->fx_<<endl;
//        cout<<ref_->camera_->fy_<<endl;
//        cout<< ref_->camera_->cx_<<endl;
//        cout<<ref_->camera_->cy_<<endl;
//        cout<<K<<endl;

        //// prepare output of  pnp_ransac
        //// rotation vector, translation vector, Output vector that contains indices of inliers in objectPoints and imagePoints .
        Mat rvec, tvec, inliners;

        //// Finds an object pose from 3D-2D point correspondences using the RANSAC scheme.
        cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliners);
        num_inliers_ = inliners.rows;
        cout<<"pnp ransac inliers: "<< num_inliers_<<endl;

        //// Save OpenCV Mat --> SE3 lie group in sophus
        T_c_r_estimated_ = SE3(
                SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
                Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
        );

//        cout<<" rvec "<<rvec.at<double>(0,0)<<" "<<rvec.at<double>(1,0)<<" "<<rvec.at<double>(2,0)<<" "<<endl;
//        cout<<" tvec "<<tvec.at<double>(0,0)<<" "<<tvec.at<double>(1,0)<<" "<<tvec.at<double>(2,0)<<" "<<endl;
        cout<<"T_c_r_estimated_: "<<T_c_r_estimated_<<endl;

        // using bundle adjustment to optimize the pose
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>;
        Block* solver_ptr = new Block( linearSolver );
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr);
        g2o::SparseOptimizer optimizer;

        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setId(0);
        pose->setEstimate( g2o::SE3Quat(
                T_c_r_estimated_.rotation_matrix(),
                T_c_r_estimated_.translation()
        ));
        optimizer.addVertex( pose );

        // edges
        for ( int i=0; i < inliners.rows; i++ )
        {
            int index = inliners.at<int>(i,0);
            // 3d->2d projection
            EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
            edge->setId(i);
            edge->setVertex(0, pose);
            edge->camera_ = curr_->camera_.get();
            edge->point_ = Vector3d( pts3d[index].x, pts3d[index].y, pts3d[index].z );
            edge->setMeasurement( Vector2d(pts2d[index].x, pts2d[index].y));
            edge->setInformation( Eigen::Matrix2d::Identity());
            optimizer.addEdge(edge);
        }

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        T_c_r_estimated_ = SE3(
                pose->estimate().rotation(),
                pose->estimate().translation()
        );



    }

    bool VisualOdometry::checkEstimatePose()
    {
        // check is the estimated pose is good
        if ( num_inliers_ < min_inliers_)
        {
            cout<<" reject this pnp ransac slove because inlier is too small:"<< num_inliers_<<endl;
            return false;
        }
        Sophus::Vector6d d = T_c_r_estimated_.log();
        if ( d.norm() > 5.0 )
        {
            cout<<"reject because motion is too large: "<<d.norm()<<endl;
            return false;
        }
        return true;
    }

    bool VisualOdometry::checkKeyFrame()
    {
        Sophus::Vector6d d = T_c_r_estimated_.log();    // lie group --> lie algebra
        Vector3d trans = d.head<3>();                   // the first n coeffs in d, Eigen function
        Vector3d rot = d.tail<3>();                     // // the last n coeffs in d, Eigen function
        if ( rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void VisualOdometry::addKeyFrame()
    {
        cout<<" add a key frame "<<endl;
        map_->insertKeyFrame( curr_ );
    }

}
