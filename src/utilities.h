//
// Created by Maria Ruxandra Robu on 13/05/2016.
//

#ifndef NEXTVIEWPLANNING_UTILITIES_H
#define NEXTVIEWPLANNING_UTILITIES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "PointCloud.h"
#include "ANN/ANN.h"


namespace nvp {
    //*************** Basic Framework *******************

    Eigen::Matrix4d createRotationMatrix(double rotRadiansX,
                                         double rotRadiansY,
                                         double rotRadiansZ);

    void getNearestPointsToCamera(Eigen::MatrixXd &projectedPts,
                                  Eigen::MatrixXd &out_nearestProjectedPts,
                                  int zbufferSideSize = 100);

    void mergePointClouds(PointCloud &pc1,
                          PointCloud &pc2,
                          Eigen::MatrixXd &out_pointSet);

    void mergePointCloudsNoDuplicates(PointCloud &pc,
                                      Eigen::MatrixXd &points_all_scans);

    void printScoreToConsole(double score);

    //*************** Individual Section *******************

    void computeNormals(Eigen::MatrixXd &pEIG,
                        Eigen::MatrixXd &pNormals,
                        int kNN = 10);

    std::vector<Camera> getKplus1ViewVector(std::vector<Camera> &kViewVect,
                                            Camera &kplus1View);

    double getMaxFromEigVector(Eigen::VectorXd& inputVec,
                               int& idxMax);


}//namespace nvp


#endif //NEXTVIEWPLANNING_UTILITIES_H
