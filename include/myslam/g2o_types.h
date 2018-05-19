//
// Created by chiliu on 5/12/18.
//

#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "common_include.h"
#include "Camera.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
//
//class g2o_types {
//
//};

namespace myslam
{
    class EdgeProjectXYZRGBD : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        virtual void computeError();
        virtual void linearizeOplus();
        virtual bool read( std::istream& in ){}
        virtual bool write( std::ostream& out) const {}
    };

    class EdgeProjectXYZRGBDPoseOnly: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        virtual void computeError();
        virtual void linearizeOplus();
        virtual bool read( std::istream& in ){}
        virtual bool write( std::ostream& out) const {}

        Vector3d point_;

    };

    // first version g2o
    class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        virtual void computeError();
        virtual void linearizeOplus();
        virtual bool read( std::istream& in ){}
        virtual bool write( std::ostream& out) const {}

        Vector3d point_;
        Camera* camera_;
    };
}


#endif //MYSLAM_G2O_TYPES_H
