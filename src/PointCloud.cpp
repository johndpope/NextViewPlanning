//
// Created by Maria Ruxandra Robu on 12/05/2016.
//
#include <iostream>
#include "PointCloud.h"
#include "converter.h"


namespace nvp {
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
            // vectors are represented as matrices in Eigen
            // .col = block operation for matrices and arrays
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

        Eigen::Vector3d centroid = m_vertices.colwise().mean();

        std::cout << "Centroid coords are: (" << centroid[0]
        << "," << centroid[1] << "," << centroid[2] << ")" << std::endl;

        //calculate the distance to the furthest point in the cloud
        for (auto row = 0; row < m_vertices.rows(); ++row) {
            curr_vertex = m_vertices.row(row);

            xSqr = (centroid[0] - curr_vertex[0]) * (centroid[0] - curr_vertex[0]);
            ySqr = (centroid[1] - curr_vertex[1]) * (centroid[1] - curr_vertex[1]);
            zSqr = (centroid[2] - curr_vertex[2]) * (centroid[2] - curr_vertex[2]);

            xyzSqr = xSqr + ySqr + zSqr;
            max_distance = sqrt(xyzSqr);

            if (max_distance > distance)
                distance = max_distance;

        }

        std::cout << "Distance from centroid to furthest vertex is " << distance << std::endl;
        radius = distance * 2;
        std::cout << "Radius is twice the distance to furthest vertex:" << radius << std::endl;
        return radius;
    }

    void PointCloud::getCenterXY(double& x, double& y) {
        Eigen::Vector3d meanP = m_vertices.colwise().mean();
        x = meanP[0];
        y = meanP[1];
    }


//    void PointCloud::demeanPointCloud() {
//        Eigen::Vector3d meanP = m_vertices.colwise().mean();
//
//        m_vertices = m_vertices - meanP.replicate(1,m_numPoints);
//    }


}//namespace nvp




