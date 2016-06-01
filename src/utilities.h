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
    //*************** Core Framework *******************

    // This function creates a rotation matrix by specifying the
    // rotation angle in radians around each of the axis
    Eigen::Matrix4d createRotationMatrix(double rotRadiansX,
                                         double rotRadiansY,
                                         double rotRadiansZ);

    // This function is meant to map a value from
    // range[currentMin,currentMax] to [rangeMin,rangeMax]
    // It is currently used to find a place in the zBuffer
    // for each point in the point cloud
    int getValueInRange(double value,
                        double currentMin,
                        double currentMax,
                        double rangeMin,
                        double rangeMax);

    // This function decides if a point is closer to the camera
    // than another based on their depths
    bool isPointCloser(double newZ,
                       double currentZ);

    // This function creates a zBuffer and an index buffer which
    // show the closest points to the camera given a resolution of zbufferSideSize
    // Set zbufferSideSize to be bigger for more complex PCD
    void createZBuffer(Eigen::MatrixXd &ptsCoord,
                       Eigen::MatrixXd &zBuffer,
                       Eigen::MatrixXd &idxBuffer,
                       int &numNearestPts,
                       int zbufferSideSize);

    // This function simulates a scan from a viewpoint by processing the
    // values from the zbuffer
    void getNearestPointsToCamera(Eigen::MatrixXd &projectedPts,
                                  Eigen::MatrixXd &out_nearestProjectedPts,
                                  int zbufferSideSize = 100);

    // This function merges 2 point clouds by simply concatenating their matrices
    // Improtant: there will be duplicates
    // Author: Karina Mady
    void mergePointClouds(PointCloud &pc1,
                          PointCloud &pc2,
                          Eigen::MatrixXd &out_pointSet);

    // This function merges two point clouds and removes any duplicates
    // Author: Karina Mady
    void mergePointCloudsNoDuplicates(PointCloud &pc1,
                                      Eigen::MatrixXd &out_reconstruction);

    // This function prints to the console an evaluation of the final score
    // obtained from the comparison with the ground truth model
    // Author: Karina Mady
    void printScoreToConsole(double score);

    //*************** Individual Section *******************

    // This function estimates the normal at every point in the
    // point cloud by fitting a plane to their neighbourhood
    // ANN is used for the neighbourhood search
    // Set kNN to the number of neighbours you want to include in a plane
    void computeNormals(Eigen::MatrixXd &pEIG,
                        Eigen::MatrixXd &pNormals,
                        int kNN = 10);

    // This function outputs a k+1 Camera vector by adding the k+1 view to
    // the k Camera vector
    // !!! The output vector is a deep copy
    std::vector<Camera> getKplus1ViewVector(std::vector<Camera> &kViewVect,
                                            Camera &kplus1View);

    // This function outputs the maximum value and its corresponding index
    // from an Eigen vector
    double getMaxFromEigVector(Eigen::VectorXd &inputVec,
                               int &idxMax);

    // This function outputs the minimum value and its corresponding index
    // from an Eigen vector
    double getMinFromEigVector(Eigen::VectorXd &inputVec,
                               int &idxMin);

    // This function adds Camera positions and orientations in a point cloud
    // with normals to visualize in Meshlab
    // !!! This function writes the point cloud directly to a .ply file
    void writePCDAndCamera(PointCloud &pc,
                           std::vector<Camera> &cameraVect);


}//namespace nvp


#endif //NEXTVIEWPLANNING_UTILITIES_H
