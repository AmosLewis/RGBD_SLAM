//
// Created by chiliu on 5/12/18.
//

#include "../include/myslam/Map.h"

namespace myslam
{
    void Map::insertKeyFrame( Frame::Ptr frame)
    {
        cout<<"Key Frame Size = "<< keyframes_.size()<<endl;
        if ( keyframes_.find(frame->id_) == keyframes_.end())
        {
            keyframes_.insert( std::make_pair(frame->id_,frame));
        }
        else
        {
            keyframes_[frame->id_] = frame;
        }
    }

    void Map::insertMapPoint( MapPoint::Ptr map_point)
    {
        if ( map_points_.find(map_point->id_) == map_points_.end())
        {
            map_points_.insert( std::make_pair(map_point->id_,map_point));
        }
        else
        {
            map_points_[map_point->id_] = map_point;
        }
    }
}