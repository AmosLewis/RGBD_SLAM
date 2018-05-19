//
// Created by chiliu on 5/12/18.
//

#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "common_include.h"

namespace myslam
{
    class Config
    {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config(){}  // private construction makes a singleton
    public:
        ~Config();  // close the file when deconstructing

        // set a new config file
        static  void setParameterFile( const std::string& filename);

        // access the parameter values
        template< typename T >
        static T get( const std::string& key)
        {
            return T( Config::config_->file_[key]);
        }
    };

}
#endif //MYSLAM_CONFIG_H
