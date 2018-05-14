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

#include "../include/myslam/config.h"
#include "../include/myslam/visual_odometry.h"

int main (int argc, char** argv)
{
    if ( argc != 2)
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }
}

