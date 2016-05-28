//
// Created by Maria Ruxandra Robu on 12/05/2016.
//

#ifndef NEXTVIEWPLANNING_POINTCLOUD_H
#define NEXTVIEWPLANNING_POINTCLOUD_H
#include <Eigen/Dense>
#include "defs.h"
#include "Camera.h"


namespace nvp {

    class PointCloud {
    public:
        PointCloud(std::string filename);
        PointCloud(Eigen::MatrixXd& in_pcd);
        PointCloud& operator= (const PointCloud &pSrc);
        PointCloud(const PointCloud&);
        ~PointCloud(){}
        void getPoints(Eigen::MatrixXd& out_pointSet);
        void setPoints(Eigen::MatrixXd& in_pointSet);
        void computeProjectedCoordinates(Camera& camera);
        void applyTransformation(Eigen::Matrix4d transfMat);
        int write(std::string filename);
        void getCenterXY(double& x, double& y);
        void getCartesianCoordinates(Eigen::MatrixXd& cartCoord);
        void computeWorldCoordinates(Camera& camera);
        void computeNearestProjectedPts(Camera &camera);
        void setNormals();

        double computeRadiusFromCentroid();
        long m_numPoints;
    private:
        Eigen::MatrixXd m_vertices; // 3xnumPoints
        Eigen::MatrixXd m_normals; // 3xnumPoints



    };
}//namespace nvp


#endif //NEXTVIEWPLANNING_POINTCLOUD_H
