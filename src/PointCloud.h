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

        PointCloud();
        PointCloud(std::string filename);
        PointCloud& operator= (const PointCloud &pSrc);
        PointCloud(const PointCloud&);
        ~PointCloud(){}

        void getPoints(Eigen::MatrixXd& out_pointSet);
        void setPoints(Eigen::MatrixXd& in_pointSet);
        void computeProjectedCoordinates(Camera& camera);
        void applyTransformation(Eigen::Matrix4d transfMat);
        void getCenterXY(double& x, double& y);
        void clipPointsByZ();
        void getCartesianCoordinates(Eigen::MatrixXd& cartCoord);
        void computeWorldCoordinates(Camera& camera);
        void computeNearestProjectedPts(Camera &camera);

        double computeRadiusFromCentroid();
        void   writeCloud(std::string filename);
        void   mergeClouds( Eigen::MatrixXd &points_all_scans);

        long m_numPoints;
    private:
        Eigen::MatrixXd m_vertices; // 3xnumPoints
        std::vector<Eigen::Vector3d> colors = { {255.,0.,0.}, {0.,255.,0.}, {0.,0.,255.}, {255.,255.,0.}, {255.,0.,255.}, {0.,255.,255.} };


    };
}//namespace nvp


#endif //NEXTVIEWPLANNING_POINTCLOUD_H
