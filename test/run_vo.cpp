//
// Created by chiliu on 5/13/18.
// test myslam
//

#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Config.h"
#include "../include/myslam/visual_odometry.h"

int main (int argc, char** argv)
{
    if ( argc != 2)
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }
    cout<< argv[1]<<endl;

    myslam::Config::setParameterFile( argv[1]);

    string dataset_dir = myslam::Config::get<string> ("dataset_dir");
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin( dataset_dir + "/associate.txt");
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;

    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        // cout<<rgb_time<<" "<<rgb_file<<" "<<depth_time<<" "<<depth_file<<endl;
        rgb_times.push_back ( atof( rgb_time.c_str() ) );
        depth_times.push_back ( atof( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir + "/" + rgb_file);
        depth_files.push_back ( dataset_dir + "/" + depth_file );

        if ( fin.good() == false)   // Check whether state of stream is good
        {
            break;
        }
    }

    myslam::Camera::Ptr camera( new myslam::Camera );

    // visualization
    cv::viz::Viz3d vis( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos (0, -1.0, -1.0 ), cam_focal_point ( 0,0,0), cam_y_dir (0,1,0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir);
    vis.setViewerPose( cam_pose);

    world_coor.setRenderingProperty( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor);

    cout<<" read total "<<rgb_files.size()<<" entries "<<endl;
    for ( int i=0; i < rgb_files.size(); i++)
    {
        cout<<"**********loop "<<i<<" *********** "<<endl;
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i] );
        if ( color.data == nullptr || depth.data == nullptr )
        {
            break;
        }
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        cout<<"cost time: "<<timer.elapsed()<<endl;
        break;
    }
}

