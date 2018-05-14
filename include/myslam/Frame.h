//
// Created by chiliu on 5/12/18.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "common_include.h"
#include "Camera.h"

namespace myslam
{
    class Frame
    {
    public:
        typedef std::shared_ptr<Frame> Ptr;
        unsigned long                  id_;             // id of frame
        double                         time_stamp_;     // when it is recorded
        SE3                            T_c_w_;          // transform from world to camera
        Camera::Ptr                    camera_;         // Pinhole RGBD Camera model
        Mat                            color_, depth_;  // color and depth image
        bool                           is_key_frame_;   // weather it is a key-frame

    public: // data members
        Frame();
        Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera= nullptr, Mat color=Mat(), Mat depth=Mat());
        ~Frame();

        // factory function
        static Frame::Ptr createFrame();

        // input all of the key points
        // find the depth in depth map
        double findDepth( const cv::KeyPoint& kp);

        // get camera center
        Vector3d getCamCenter() const;  //cannot change no mutable member variables

        void setPose( const SE3& T_c_w);

        //check is a points is in this frame
        bool isInFrame( const Vector3d& pt_world );
    };

}



#endif //MYSLAM_FRAME_H
