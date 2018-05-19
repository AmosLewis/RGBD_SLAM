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
        cout<<"VO get config finish"<<endl;
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

//                v1, v2
//                // compute the 3d position of features in ref frame
//                setRef3DPoints();

                // v3
                addKeyFrame(); // the first frame is a key-frame
                cout<<"INITIALIZING addFrame finish"<<endl;
                break;
            }
            case OK:
            {
                curr_ = frame;

                // v3
                curr_->T_c_w_ = ref_->T_c_w_;

                extractKeyPoints();                 // orb->detect
                computeDescriptors();               // orb->compute
                featureMatching();                  // calculate good matches
                poseEstimationPnP();                // calculate T_c_w_estimated_
                if (checkEstimatePose() == true )   // this is a good estimation
                {
//                    v1 v2
//                    // T_c_w = T_c_r * T_r_w
//                    curr_->T_c_w_ = T_c_w_estimated_ * ref_->T_c_w_;
//                    ref_ = curr_;
//                    setRef3DPoints();

                    // v3
                    curr_->T_c_w_ = T_c_w_estimated_;
                    optimizeMap();

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
                    cout<<"OK addFrame false"<<endl;
                    return false;
                }
                cout<<"OK addFrame finish"<<endl;
                break;
            }
            case LOST:
            {
                cout<<" vo has lost."<<endl;
                cout<<"LOST addFrame false"<<endl;
                break;
            }
        }
        cout<<"addFrame SWITCH DONE"<<endl;
        return true;
    }

    void VisualOdometry::extractKeyPoints()
    {
        orb_->detect ( curr_->color_, keypoints_curr_);
        cout<<"extractKeyPoints finish "<<endl;
    }

    void VisualOdometry::computeDescriptors()
    {
        orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_);
        cout<<"computeDescriptors finish "<<endl;
    }

    void VisualOdometry::featureMatching()
    {
        boost::timer timer;
        // match desp_ref and desp_curr, use OpencCV's burte force match
        vector<cv::DMatch> matches;

//        v1, v2 test brute-force matcher
//        cv::BFMatcher matcher (cv::NORM_HAMMING);
//        matcher.match( descriptors_ref_, descriptors_curr_, matches );

        // v3 flann matcher
        // select the candidates in map
        Mat desp_map;
        vector<MapPoint::Ptr> candidate;
        for ( auto& allpoints: map_->map_points_ )
        {
            MapPoint::Ptr& p = allpoints.second;
            // check if the mappoint in the current map
            if ( curr_->isInFrame(p->pos_))
            {
                // add to candidate
                p->visible_times_++;
                candidate.push_back( p );
                desp_map.push_back( p->descriptor_);
            }
        }
        matcher_flann_.match( desp_map, descriptors_curr_, matches);


        // select good matches
        //// find minimum distance in all matches
        float min_dis = std::min_element(
                matches.begin(), matches.end(),
                []( const cv::DMatch& m1, const cv::DMatch& m2 )
                {
                    return m1.distance < m2.distance;
                }
        )->distance;

        // v1 v2 match
//        ////  store good matches
//        feature_matches_.clear();
//        for( cv::DMatch& m: matches )
//        {
//            if ( m.distance < max<float>( min_dis * match_ratio_, 30.0 ) )
//            {
//                feature_matches_.push_back(m);
//            }
//        }
//        cout<< " Brute Force good matches: "<<feature_matches_.size()<<endl;
//        cout<<"Brute Force match cost time: "<<timer.elapsed()<<endl;

        // v3 match
        ////  store good matches
        match_3dpts_.clear();
        match_2dkp_index_.clear();
        for( cv::DMatch& m: matches )
        {
            if ( m.distance < max<float>( min_dis * match_ratio_, 30.0 ) )
            {
                match_3dpts_.push_back( candidate[m.queryIdx] );
                match_2dkp_index_.push_back( m.trainIdx);
            }
        }
        cout<< "Flann good matches: "<<match_3dpts_.size()<<endl;
        cout<<"Flann match cost time: "<<timer.elapsed()<<endl;
        cout<<"featureMatching finish"<<endl;
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
        cout<< "setRef3DPoints finish"<<endl;

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
        T_c_w_estimated_ = SE3(
                SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
                Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
        );

//        cout<<" rvec "<<rvec.at<double>(0,0)<<" "<<rvec.at<double>(1,0)<<" "<<rvec.at<double>(2,0)<<" "<<endl;
//        cout<<" tvec "<<tvec.at<double>(0,0)<<" "<<tvec.at<double>(1,0)<<" "<<tvec.at<double>(2,0)<<" "<<endl;
        cout<<"T_c_w_estimated_: "<<T_c_w_estimated_<<endl;

        // using bundle adjustment to optimize the pose
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>;
        Block* solver_ptr = new Block( linearSolver );
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr);
        g2o::SparseOptimizer optimizer;

        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setId(0);
        pose->setEstimate( g2o::SE3Quat(
                T_c_w_estimated_.rotation_matrix(),
                T_c_w_estimated_.translation()
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

        T_c_w_estimated_ = SE3(
                pose->estimate().rotation(),
                pose->estimate().translation()
        );
        cout<<"poseEstimationPnP finish "<<endl;
    }

    bool VisualOdometry::checkEstimatePose()
    {
        // check is the estimated pose is good
        if ( num_inliers_ < min_inliers_)
        {
            cout<<" checkEstimatePose finsh: reject this pnp ransac slove because inlier is too small:"<< num_inliers_<<endl;
            return false;
        }
        Sophus::Vector6d d = T_c_w_estimated_.log();
        if ( d.norm() > 5.0 )
        {
            cout<<"checkEstimatePose finsh: reject because motion is too large: "<<d.norm()<<endl;
            return false;
        }
        cout<<"checkEstimatePose finish true "<<endl;
        return true;
    }

    bool VisualOdometry::checkKeyFrame()
    {
        Sophus::Vector6d d = T_c_w_estimated_.log();    // lie group --> lie algebra
        Vector3d trans = d.head<3>();                   // the first n coeffs in d, Eigen function
        Vector3d rot = d.tail<3>();                     // // the last n coeffs in d, Eigen function
        if ( rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans)
        {
            cout<<"checkKeyFrame true "<<endl;
            return true;
        }
        else
        {
            cout<<"checkKeyFrame False "<<endl;
            return false;
        }
    }

    void VisualOdometry::addKeyFrame()
    {
//        v1 v2 version
//        cout<<" add a key frame "<<endl;
//        map_->insertKeyFrame( curr_ );

        // v3 version
        if ( map_->keyframes_.empty())
        {
            // first key-frame, add all 3d points into map
            for ( size_t i = 0; i<keypoints_curr_.size(); i++)
            {
                double d = curr_->findDepth( keypoints_curr_[i]);
                if ( d < 0)
                {
                    continue;
                }
                Vector3d p_world = ref_->camera_->pixel2world(
                        Vector2d( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), curr_->T_c_w_, d
                );
                Vector3d n = p_world - ref_->getCamCenter();
                n.normalize();
                MapPoint::Ptr map_point = MapPoint::createMapPoint(
                        p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
                );
                map_->insertMapPoint( map_point);
            }
        }

        map_->insertKeyFrame( curr_ );
        ref_ = curr_;
        cout<<"addKeyFrame finish"<<endl;
    }

    // v3
    void VisualOdometry::optimizeMap()
    {
        // reomve the hardly seen and no visible points
        for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end();)
        {
            if ( !curr_->isInFrame(iter->second->pos_))
            {
                iter = map_->map_points_.erase(iter);
                continue;
            }

            float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
            if ( match_ratio < map_point_erase_ratio_)
            {
                iter = map_->map_points_.erase(iter);
                continue;
            }

            double angle = getViewAngle( curr_, iter->second);
            if ( angle > M_PI/6)
            {
                iter = map_->map_points_.erase(iter);
                continue;
            }

            // TODO triangulate map point
            iter++;
        }

        if ( match_2dkp_index_.size()<100 )
            addMapPoints();
        if ( map_->map_points_.size()>1000)
        {
            // TODO map is too large, remove some one
            cout<<"map is too large, remove some one"<<endl;
            map_point_erase_ratio_ +=0.05;
        }
        else
        {
            map_point_erase_ratio_ = 0.1;
        }

        cout<<"map points:  "<<map_->map_points_.size()<<endl;
        cout<<"optimizeMap finish"<<endl;
    }

    // v3
    void VisualOdometry::addMapPoints()
    {
        // add the new map points into map
        vector<bool> matched(keypoints_curr_.size(), false);
        for (int index:match_2dkp_index_)
        {
            matched[index] = true;
        }
        for ( int i=0; i<keypoints_curr_.size(); i++)
        {
            if ( matched[i] == true )
            {
                continue;
            }
            double d = ref_->findDepth( keypoints_curr_[i]);
            if( d<0 )
            {
                continue;
            }
            Vector3d p_world = ref_->camera_->pixel2world(
                Vector2d( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),
                curr_->T_c_w_,d
            );
            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                    p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
            );
            map_->insertMapPoint( map_point );
        }
        cout<<"addMapPoints finish"<<endl;
    }

    // v3
    double VisualOdometry::getViewAngle(Frame::Ptr frame, MapPoint::Ptr point)
    {
        Vector3d n = point->pos_ - frame->getCamCenter();
        n.normalize();
        cout<<"getViewAngle finish"<<endl;
        return acos( n.transpose()*point->norm_);
    }
}
