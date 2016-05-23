//
// Created by Maria Ruxandra Robu on 12/05/2016.
//
#include <iostream>
#include "PointCloud.h"
#include "converter.h"
#include "utilities.h"


namespace nvp {
//    struct eigenVecZComp{
//        bool operator()(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2){
//            return vec1[2] > vec1[2];
//        }
//    } ComparisonFunc;

    PointCloud::PointCloud(std::string filename) {
        MyMesh mesh;
        if (!OpenMesh::IO::read_mesh(mesh, filename)) {
            std::cerr << "Error: Cannot read mesh from " << filename << std::endl;
        }
        m_numPoints = mesh.n_vertices();
        m_vertices = Eigen::MatrixXd::Zero(3, m_numPoints);
        uint32_t colIdx = 0;

        for (MyMesh::VertexIter v_it = mesh.vertices_begin();
             v_it != mesh.vertices_end(); ++v_it) {
            OpenMesh::Vec3d thisOMVert = mesh.point(*v_it);
            Eigen::Vector3d thisEVert = convertOMVecToEIGENVec(thisOMVert);

            m_vertices.col(colIdx) = thisEVert;
            colIdx++;
        }
    }

    void PointCloud::getPoints(Eigen::MatrixXd &out_pointSet) {
//        out_pointSet = Eigen::MatrixXd::Zero(3,m_numPoints);
        out_pointSet = m_vertices;
    }

    void PointCloud::setPoints(Eigen::MatrixXd &in_pointSet) {
        m_vertices = in_pointSet;
        m_numPoints = in_pointSet.cols();
    }


    int PointCloud::write(std::string filename) {
        MyMesh mesh;
        uint32_t colIdx = 0;
        for (int i = 0; i < m_numPoints; i++) {
            OpenMesh::Vec3d thisVert = convertEIGENVecToOMVec(m_vertices.col(colIdx));
            mesh.add_vertex(thisVert);
            colIdx++;
        }
        std::cout << mesh.n_vertices() << " points written\n";

        if (!OpenMesh::IO::write_mesh(mesh, filename)) {
            std::cerr << "Error: cannot write mesh to " << filename << std::endl;
            return ERROR;
        }
        return SUCCESS;
    }

    void PointCloud::applyTransformation(Eigen::Matrix4d transfMat) {
        for (int i = 0; i < m_numPoints; i++) {
            Eigen::Vector4d thisCoord = m_vertices.col(i).homogeneous();
            Eigen::Vector4d updCoord = transfMat * thisCoord;
            // save the transformed coordinates
            m_vertices.col(i) = updCoord.head(3);
        }
    }

    PointCloud &PointCloud::operator=(const PointCloud &pSrc) {
        // check for self - assignment
        if (this == &pSrc)
            return *this;

        m_vertices = pSrc.m_vertices;
        m_numPoints = pSrc.m_numPoints;

        // return the existing object
        return *this;
    }

    PointCloud::PointCloud(const PointCloud &pSrc) {
        m_vertices = pSrc.m_vertices;
        m_numPoints = pSrc.m_numPoints;
    }

    double PointCloud::computeRadiusFromCentroid() {
        // Author: Karina Mady
        double distance = 0;
        double max_distance = 0;
        double xSqr;
        double ySqr;
        double zSqr;
        double xyzSqr;
        Eigen::Vector3d curr_vertex;
        double radius = 0;

        Eigen::Vector3d centroid = m_vertices.rowwise().mean();

//        std::cout << "Centroid coords are: (" << centroid[0]
//        << "," << centroid[1] << "," << centroid[2] << ")" << std::endl;

        //calculate the distance to the furthest point in the cloud
        for (auto colIdx = 0; colIdx < m_vertices.rows(); ++colIdx) {
            curr_vertex = m_vertices.col(colIdx);

            xSqr = (centroid[0] - curr_vertex[0]) * (centroid[0] - curr_vertex[0]);
            ySqr = (centroid[1] - curr_vertex[1]) * (centroid[1] - curr_vertex[1]);
            zSqr = (centroid[2] - curr_vertex[2]) * (centroid[2] - curr_vertex[2]);

            xyzSqr = xSqr + ySqr + zSqr;
            max_distance = sqrt(xyzSqr);

            if (max_distance > distance)
                distance = max_distance;

        }

//        std::cout << "Distance from centroid to furthest vertex is " << distance << std::endl;
        radius = distance * 2;
//        std::cout << "Radius is twice the distance to furthest vertex:" << radius << std::endl;
        return radius;
    }

    void PointCloud::getCenterXY(double &x, double &y) {
        Eigen::Vector3d meanP = m_vertices.colwise().mean();
        x = meanP[0];
        y = meanP[1];
    }


    void PointCloud::clipPointsByZ() {
        double minZ = m_vertices.row(2).minCoeff();
        double maxZ = m_vertices.row(2).maxCoeff();
//        std::cout << "Camera - PointCloud: Min distance = " << minZ << std::endl;
//        std::cout << "Camera - PointCloud: Max distance = " << maxZ << std::endl;

        double threshold = minZ + std::abs(maxZ - minZ) / 2.0;
//      std::cout << "Camera - PointCloud: Threshold = " << threshold << std::endl;

        Eigen::MatrixXd clippedPoints(Eigen::MatrixXd::Zero(3,m_numPoints));
        long colIdx = 0;
        std::cout << "Remove points that are further away than the threshold: " << threshold << std::endl;

        for (int i = 0; i < m_vertices.cols(); i++) {
            if (m_vertices.col(i)[2] < threshold) {
                clippedPoints.col(colIdx) = m_vertices.col(i);
                colIdx ++;
            }
        }
        m_vertices = clippedPoints.block(0,0,3,colIdx-1);
        m_numPoints = m_vertices.cols();
    }

    void PointCloud::getCartesianCoordinates(Eigen::MatrixXd &cartCoord) {
        cartCoord = Eigen::MatrixXd::Zero(2, m_numPoints);

        // x coord / z coord
        cartCoord.row(0) = (m_vertices.row(0).array() /
                            m_vertices.row(2).replicate(3, 1).array()).matrix();

        // y coord / z coord
        cartCoord.row(1) = (m_vertices.row(1).array() /
                            m_vertices.row(2).replicate(3, 1).array()).matrix();

    }

    void PointCloud::computeWorldCoordinates(Camera &camera) {
        // return the world coordinated of the projected points
        // by applying the inverse of the camera matrix
        Eigen::Matrix4d worldToCamera;
        camera.getCameraTransform(worldToCamera);

        this->applyTransformation(worldToCamera.inverse());
    }

    void PointCloud::computeProjectedCoordinates(Camera &camera) {
        // this method projects all the points in the view of the camera
        // it doesn't check for depth
        Eigen::Matrix4d worldToCamera;
        camera.getCameraTransform(worldToCamera);

        this->applyTransformation(worldToCamera);
    }

    void PointCloud::computeNearestProjectedPts(Camera &camera) {
        // this method projects all of the points in the view of the camera and
        // it saves only the ones that are first seen with a z-buffer

        computeProjectedCoordinates(camera);

        Eigen::MatrixXd nearestProjectedPts;

        getNearestPointsToCamera(m_vertices,
                                 nearestProjectedPts);

        m_vertices = nearestProjectedPts;
        m_numPoints = m_vertices.cols();

    }









//    void PointCloud::demeanPointCloud() {
//        Eigen::Vector3d meanP = m_vertices.colwise().mean();
//
//        m_vertices = m_vertices - meanP.replicate(1,m_numPoints);
//    }


}//namespace nvp




