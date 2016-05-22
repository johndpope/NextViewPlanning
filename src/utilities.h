//
// Created by Maria Ruxandra Robu on 13/05/2016.
//

#ifndef NEXTVIEWPLANNING_UTILITIES_H
#define NEXTVIEWPLANNING_UTILITIES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace nvp{
    Eigen::Matrix4d createRotationMatrix(double rotRadiansX,
                                         double rotRadiansY,
                                         double rotRadiansZ);

    void getNearestPointsToCamera(Eigen::MatrixXd &projectedPts,
                                  Eigen::MatrixXd &out_nearestProjectedPts);

//    Eigen::MatrixXd sortMatrixByZ(Eigen::MatrixXd& coordMat);
//
//    bool isZBiggerThan(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2);
//
//    void swapVectors(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2);
}//namespace nvp


#endif //NEXTVIEWPLANNING_UTILITIES_H
