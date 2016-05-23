//
// Created by Maria Ruxandra Robu on 12/05/2016.
//
#include <iostream>
#include <fstream>
#include "PointCloud.h"
#include "converter.h"
#include "utilities.h"
#include "time.h"

namespace nvp {


    PointCloud::PointCloud(){}

    PointCloud::PointCloud(std::string filename)
    {
        std::ifstream f(filename);
        if ( !f.is_open() )
        {
            std::cerr << "[PointCloud.cpp::PointCloud()] could not open " << filename << std::endl;
            exit(1);
        }
        std::vector<Eigen::Vector3d> vertData; // filled with read data

        m_numPoints = -1;
        long int vertexId(0);
        bool endHeaderFound(false);
        std::string line;
        while ( std::getline(f,line) && (vertexId != m_numPoints) )
        {
            if ( endHeaderFound )
            {
                std::istringstream iss(line);
                Eigen::Vector3d pt;
                iss >> pt(0) >> pt(1) >> pt(2);
                vertData.push_back(pt);
                ++vertexId;
                continue;
            }
            // find vertex count, then end of header
            static const std::string tag("element vertex");
            size_t index = line.find(tag);
            if ( index != std::string::npos )
            {
                m_numPoints = std::atol( line.substr(index+tag.size(),line.size()-index+tag.size()).c_str() );
                std::cout << "Vertex count: " << m_numPoints << std::endl;
                vertData.reserve(m_numPoints);
            }
            else if ( line.find("end_header") != std::string::npos )
            {
                if ( m_numPoints == (long int)(-1) )
                {
                    std::cerr << "[PointCloud.cpp::PointCloud()] reached end of ply header, but did not find vertex count...stopping" << std::endl;
                    exit(1);
                }
                endHeaderFound = true;
            }
            else
            { /*std::cout << "[PointCloud.cpp::PointCloud()] skipping line " << line << std::endl;*/}
        }

        m_vertices.resize( 3, vertData.size() );
        //std::cout<<"m_vertices.cols() is "<<m_vertices.cols()<<std::endl;
        //std::cout<<"m_numPoints should be the same as above: "<<m_numPoints<<std::endl;

        for ( size_t pntId = 0; pntId != vertData.size(); ++pntId )
        {
            m_vertices.col(pntId) = vertData[ pntId ];
        }
        f.close();
        std::cout << "Cloud has been read in" << std::endl;

    }

    void PointCloud::writeCloud(std::string filename)
    {
        srand(time(NULL));
        int value = rand()%100;
        Eigen::Vector3d color = colors[value % colors.size()];

        std::stringstream stream;
        stream << m_vertices.cols();
        std::string s_num_vertices = stream.str();
       // std::cout<<s_num_vertices<<std::endl;

        const std::string vertices = "element vertex " + s_num_vertices;
        //std::cout<<vertices<<std::endl;

        std::ofstream fout(filename);
        std::vector<std::string> header = {
                "ply",
                "format ascii 1.0",
                vertices,
                "property float x",
                "property float y",
                "property float z",
                "property uchar red",
                "property uchar green",
                "property uchar blue",
                "end_header"};

        if (fout.is_open())
        {
            for (unsigned i = 0; i < header.size(); i++)
            {
                fout << header[i] << "\n";
            }
            for (int colId = 0; colId < m_vertices.cols(); colId++)
            {
                std::string point;
                for (int rowId = 0; rowId < m_vertices.rows(); rowId++)
                {
                    std::stringstream coord;
                    coord << m_vertices(rowId, colId);
                    std::string s_coord = coord.str();
                    point = point + s_coord + " ";
                }
                fout << point << " " << color(0) << " " << color(1) << " " << color(2) << "\n";
            }
        }
        else //file could not be opened
            std::cout << "File could not be opened." << std::endl;
        fout.close();
    }


    void PointCloud::getPoints(Eigen::MatrixXd &out_pointSet) {
//        out_pointSet = Eigen::MatrixXd::Zero(3,m_numPoints);
        out_pointSet = m_vertices;
    }

    void PointCloud::setPoints(Eigen::MatrixXd &in_pointSet) {
        m_vertices = in_pointSet;
        m_numPoints = in_pointSet.cols();
    }


    void PointCloud::applyTransformation(Eigen::Matrix4d transfMat) {
        for (int i = 0; i < m_numPoints; i++) {
            Eigen::Vector4d thisCoord = m_vertices.col(i).homogeneous();
            Eigen::Vector4d updCoord = transfMat * thisCoord;
            // save the transformed coordinates
            m_vertices.col(i) = updCoord.head(3);
            //std::cout<<"m_vertices.col("<<i<<") is"<<m_vertices.col(i)<<std::endl;
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
        for (auto col = 0; col < m_vertices.cols(); ++col)
        {
            curr_vertex = m_vertices.col(col);

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

    void PointCloud::computeWorldCoordinates(Camera &camera)
    {
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

    void PointCloud::computeNearestProjectedPts(Camera &camera)
    {
        // this method projects all of the points in the view of the camera and
        // it saves only the ones that are first seen with a z-buffer
        computeProjectedCoordinates(camera);
        Eigen::MatrixXd nearestProjectedPts;
        getNearestPointsToCamera(m_vertices,nearestProjectedPts);
        m_vertices = nearestProjectedPts;
        m_numPoints = m_vertices.cols();
    }


    //this function is being called for every scan we want to use in the reconstruction
    //takes the point cloud from the scan and merges it with the point cloud that will be
    //written to the reconstruction file
    void PointCloud::mergeClouds( Eigen::MatrixXd &points_all_scans)
    {
        //std::cout<<"points_all_scans is "<<points_all_scans.cols()<<std::endl;
        std::cout<<"m_vertices.cols() is "<<m_vertices.cols()<<std::endl;
        points_all_scans.resize(3, points_all_scans.cols()+ m_vertices.cols());
        points_all_scans<<(points_all_scans,m_vertices);
        std::cout<<"points_all_scans after merge is "<<points_all_scans.cols()<<std::endl;

    }

}//namespace nvp




