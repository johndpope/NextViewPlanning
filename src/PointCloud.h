//
// Created by Maria Ruxandra Robu on 12/05/2016.
//

#ifndef NEXTVIEWPLANNING_POINTCLOUD_H
#define NEXTVIEWPLANNING_POINTCLOUD_H
#include <Eigen/Dense>
#include "defs.h"


namespace nvp {


    class PointCloud {
    public:
        PointCloud(std::string filename);
        PointCloud& operator= (const PointCloud &pSrc);
        PointCloud(const PointCloud&);
        ~PointCloud(){}
        void getPoints(Eigen::MatrixXd& out_pointSet);
        void setPoints(Eigen::MatrixXd& in_pointSet);
        void applyTransformation(Eigen::Matrix4d transfMat);
        int write(std::string filename);
        void getCenterXY(double& x, double& y);

        double computeRadiusFromCentroid();
        long m_numPoints;
    private:
        Eigen::MatrixXd m_vertices; // 3xnumPoints


    };
}//namespace nvp


#endif //NEXTVIEWPLANNING_POINTCLOUD_H
