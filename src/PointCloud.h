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
        //*************** Core Framework *******************

        PointCloud(std::string filename);

        PointCloud(Eigen::MatrixXd &in_pcd);

        PointCloud &operator=(const PointCloud &pSrc);

        PointCloud(const PointCloud &);

        ~PointCloud() { }

        void getPoints(Eigen::MatrixXd &out_pointSet);

        void setPoints(Eigen::MatrixXd &in_pointSet);

        // This function projects all of the points in m_vertices
        // using the input camera
        void computeProjectedCoordinates(Camera &camera);

        // This function applies a 4x4 transformation matrix to
        // the point cloud
        void applyTransformation(Eigen::Matrix4d transfMat);

        // This function writes the point cloud tot he specified path
        int write(std::string filename);

        // This function outputs the centroid of the point cloud as
        // the mean of all the points on x and y
        void getCenterXY(double &x, double &y);

        // This function outputs the cartesian coordinates obtained
        // from the previously projected points
        void getCartesianCoordinates(Eigen::MatrixXd &cartCoord);

        // This function uses the input camera to back project
        // the points back to the 3D world
        void computeWorldCoordinates(Camera &camera);

        // This function computes the nearest points seen by
        // the input camera by using a zBuffer
        void computeNearestProjectedPts(Camera &camera,
                                        int zbufferSideSize = 100);

        // This function computes the radius of the surrounding ring
        // based on the biggest distance between 2 points from the point cloud
        // Author: Karina Mady
        double computeRadiusFromCentroid();

        //*************** Individual Section *******************

        void getNormals(Eigen::MatrixXd &out_normals);

        // This function estimates and sets the normals to the point cloud - (using ANN)
        void setNormals();

        void setPointsAndNormals(Eigen::MatrixXd &in_pointSet,
                                 Eigen::MatrixXd &in_normals);


        long m_numPoints;
    private:
        Eigen::MatrixXd m_vertices; // 3xnumPoints
        Eigen::MatrixXd m_normals; // 3xnumPoints



    };
}//namespace nvp


#endif //NEXTVIEWPLANNING_POINTCLOUD_H
