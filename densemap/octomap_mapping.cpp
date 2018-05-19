#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <octomap/octomap.h>    // for octomap 

#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <boost/timer/timer.hpp>
#include <cstdio>
int main( int argc, char** argv )
{
	boost::timer timer;
	freopen("octomap_output.txt","w", stdout);

    vector<cv::Mat> colorImgs, depthImgs;    // color image and depth image
    vector<Eigen::Isometry3d> poses;         // camera pose
    
    ifstream fin("./data/pose.txt");
    if (!fin)
    {
        cerr<<"cannot find pose file"<<endl;
        return 1;
    }
    
    for ( int i=0; i<5; i++ )
    {
        boost::format fmt( "./data/%s/%d.%s" ); //image file format
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
        
        double data[7] = {0};
        for ( int i=0; i<7; i++ )
        {
            fin>>data[i];
        }
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d T(q);
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        poses.push_back( T );
    }
    
    // join point cloud
    // camera intrinsic
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    
    cout<<"convert image to Octomap ..."<<endl;
    
    // octomap tree 
    octomap::OcTree tree( 0.05 ); //  paramter is resolution
    
    for ( int i=0; i<5; i++ )
    {
        cout<<"converting image: "<<i+1<<endl; 
        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        
        octomap::Pointcloud cloud;  // the point cloud in octomap 
        
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // depth value
                if ( d==0 ) continue; // detect nothing
                if ( d >= 7000 ) continue; // depth too big, erase
                Eigen::Vector3d point; 
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                Eigen::Vector3d pointWorld = T*point;
                // put word coordinate into point cloud
                cloud.push_back( pointWorld[0], pointWorld[1], pointWorld[2] ); 
            }
        // insert pointcloud into octomap, set original in order to calculate projection line
        tree.insertPointCloud( cloud, octomap::point3d( T(0,3), T(1,3), T(2,3) ) );     
    }
    
    // update middle node and write 
    tree.updateInnerOccupancy();
    cout<<"saving octomap ... "<<endl;
    tree.writeBinary( "octomap_output.bt" );
    cout<<" used time "<<timer.elapsed()<<endl;
    return 0;
}
