//
// Created by Maria Ruxandra Robu on 13/05/2016.
//

#ifndef NEXTVIEWPLANNING_UTILITIES_H
#define NEXTVIEWPLANNING_UTILITIES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "PointCloud.h"

namespace nvp {
    Eigen::Matrix4d createRotationMatrix(double rotRadiansX,
                                         double rotRadiansY,
                                         double rotRadiansZ);

    void getNearestPointsToCamera(Eigen::MatrixXd &projectedPts,
                                  Eigen::MatrixXd &out_nearestProjectedPts);

    void mergePointClouds(PointCloud& pc1,
                          PointCloud& pc2,
                          Eigen::MatrixXd& out_pointSet);
    void mergePointCloudsNoDuplicates(PointCloud& pc,
                                      Eigen::MatrixXd &points_all_scans);

    }//namespace nvp


#endif //NEXTVIEWPLANNING_UTILITIES_H
