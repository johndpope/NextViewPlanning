//
// Created by Maria Ruxandra Robu on 12/05/2016.
//
#include <iostream>
#include "PointCloud.h"
#include "converter.h"
#include "utilities.h"


namespace nvp {

    PointCloud::PointCloud(std::string filename) {
        std::cout << "Reading 3D model from file:" << filename << "..." << std::endl;

        MyMesh mesh;
        if (!OpenMesh::IO::read_mesh(mesh, filename)) {
            std::cerr << "Error: Cannot read mesh from " << filename << std::endl;
        }
        m_numPoints = mesh.n_vertices();
        m_vertices = Eigen::MatrixXd::Zero(3, m_numPoints);
        uint32_t colIdx = 0;

        for (MyMesh::VertexIter v_it = mesh.vertices_begin();
             v_it != mesh.vertices_end(); ++v_it) {
            MyTraits::Point thisOMVert = mesh.point(*v_it);
            Eigen::Vector3d thisEVert = convertOMVecToEIGENVec(thisOMVert);

            m_vertices.col(colIdx) = thisEVert;
            colIdx++;
        }
        // std::cout << m_numPoints << " point read" << std::endl;
        m_normals = Eigen::MatrixXd::Zero(3,1);
    }

    PointCloud::PointCloud(Eigen::MatrixXd &in_pcd) {
        m_vertices = in_pcd;
        m_numPoints = in_pcd.cols();
        m_normals = Eigen::MatrixXd::Zero(3,1);
    }

    void PointCloud::getPoints(Eigen::MatrixXd &out_pointSet) {
//        out_pointSet = Eigen::MatrixXd::Zero(3,m_numPoints);
        out_pointSet = m_vertices;

    }

    void PointCloud::setPoints(Eigen::MatrixXd &in_pointSet) {
        m_vertices = in_pointSet;
        m_numPoints = in_pointSet.cols();
        m_normals = Eigen::MatrixXd::Zero(3,1);
    }
    void PointCloud::setPointsAndNormals(Eigen::MatrixXd &in_pointSet,
                                         Eigen::MatrixXd &in_normals) {
        m_vertices = in_pointSet;
        m_numPoints = in_pointSet.cols();
        m_normals = in_normals;
    }

    int PointCloud::write(std::string filename) {
        std::cout << "Writing file: " << filename << "..." << std::endl;

        MyMesh mesh;
        uint32_t colIdx = 0;
        bool NORMALS = (m_normals.col(0).prod() != 0);
        std::cout << "NORMALS flag = " << NORMALS << std::endl;

        for (int i = 0; i < m_numPoints; i++) {
            MyTraits::Point thisVert = convertEIGENVecToOMVec(m_vertices.col(colIdx));
            mesh.add_vertex(thisVert);

            colIdx++;
        }

        if (NORMALS) {
            colIdx = 0;
            for (MyMesh::VertexIter v_it = mesh.vertices_begin();
                 v_it != mesh.vertices_end(); ++v_it) {
                Eigen::Vector3d thisNormal = m_normals.col(colIdx);
                thisNormal.normalize();
                MyTraits::Normal thisOMNormal = convertEIGENVecToOMVec(thisNormal);
                // m_mesh.set_point( *v_it, thisOMVert);
                mesh.set_normal(*v_it, thisOMNormal);
                colIdx++;
            }
        }
        std::cout << mesh.n_vertices() << " points written";

        OpenMesh::IO::Options wopt;
        if (NORMALS && mesh.has_vertex_normals()) {
            std::cout << " with normals";
            wopt += OpenMesh::IO::Options::VertexNormal;
        }
        std::cout << std::endl;

        if (!OpenMesh::IO::write_mesh(mesh, filename, wopt)) {
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
        m_normals = pSrc.m_normals;

        // return the existing object
        return *this;
    }

    PointCloud::PointCloud(const PointCloud &pSrc) {
        m_vertices = pSrc.m_vertices;
        m_numPoints = pSrc.m_numPoints;
        m_normals = pSrc.m_normals;

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
        Eigen::Vector3d meanP = m_vertices.rowwise().mean();
        x = -meanP[0];
        y = -meanP[1];
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

    void PointCloud::computeNearestProjectedPts(Camera &camera,
                                                int zbufferSideSize) {
        // this method projects all of the points in the view of the camera and
        // it saves only the ones that are first seen with a z-buffer

        computeProjectedCoordinates(camera);

        Eigen::MatrixXd nearestProjectedPts;

        getNearestPointsToCamera(m_vertices,
                                 nearestProjectedPts,
                                 zbufferSideSize);

        m_vertices = nearestProjectedPts;
        m_numPoints = m_vertices.cols();

    }

    void PointCloud::setNormals() {
        int kNN = 10;
        Eigen::MatrixXd myNormals(3, m_numPoints);
        computeNormals(m_vertices, myNormals, kNN);
        m_normals = myNormals;
        //std::cout << "Normals computed = " << m_normals.cols() << std::endl;
        //std::cout << "Example:\n" << m_normals.block(0, 0, 3, 10) << std::endl;
    }

    void PointCloud::getNormals(Eigen::MatrixXd &out_normals) {
        bool NORMALS = (m_normals.col(0).prod() != 0);

        if (!NORMALS)
            this->setNormals();
        out_normals = m_normals;
    }


}//namespace nvp




