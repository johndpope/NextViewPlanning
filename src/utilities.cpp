//
// Created by Maria Ruxandra Robu on 13/05/2016.
//
#include <iostream>
#include "defs.h"
#include "utilities.h"


namespace nvp {

    Eigen::Matrix4d createRotationMatrix(double rotRadiansX,
                                         double rotRadiansY,
                                         double rotRadiansZ) {

        Eigen::Matrix4d rotationMat(Eigen::Matrix4d::Identity());
        // awesome website: http://ksimek.github.io/2012/08/22/extrinsic/


        Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(rotRadiansX,
                                                               Eigen::Vector3d(1, 0, 0)));
        Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(rotRadiansY,
                                                               Eigen::Vector3d(0, 1, 0)));
        Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(rotRadiansZ,
                                                               Eigen::Vector3d(0, 0, 1)));
        Eigen::Affine3d rotMat = rz * ry * rx;

        rotationMat = rotMat.matrix();

//        // set translations to the origin of the mesh
//        rotationMat(0, 3) = 0.0;
//        rotationMat(1, 3) = 0.0;
//        rotationMat(2, 3) = 164.0;

//        std::cout << "rotationMat: \n" << rotMat.matrix() << std::endl;
        return rotationMat;
    }

    int getValueInRange(double value,
                        double currentMin,
                        double currentMax,
                        double rangeMin,
                        double rangeMax) {
        // This function brings the *value* which is in the range [currentMin, currentMax]
        // into [rangeMin,rangeMax]

        return int((rangeMin + ((rangeMax - rangeMin) *
                                (value - currentMin)) / (currentMax - currentMin)));
    }

    bool isPointCloser(double newZ, double currentZ) {
        return newZ <= currentZ;
    }

    void createZBuffer(Eigen::MatrixXd &ptsCoord,
                       Eigen::MatrixXd &zBuffer,
                       Eigen::MatrixXd &idxBuffer,
                       int &numNearestPts) {
        numNearestPts = 0;
        double xMin = ptsCoord.row(0).minCoeff();
        double xMax = ptsCoord.row(0).maxCoeff();
        double yMin = ptsCoord.row(1).minCoeff();
        double yMax = ptsCoord.row(1).maxCoeff();
        double zMin = ptsCoord.row(2).minCoeff();
        double zMax = ptsCoord.row(2).maxCoeff();
        int widthBuffer = ZBUFFER_SIDE;
        int heightBuffer = ZBUFFER_SIDE;

        zBuffer = zMax * Eigen::MatrixXd::Ones(widthBuffer, heightBuffer);
        idxBuffer = -1 * Eigen::MatrixXd::Ones(widthBuffer, heightBuffer);

        int currXIdx, currYIdx;
        for (int i = 0; i < ptsCoord.cols(); i++) {
            currXIdx = getValueInRange(ptsCoord.col(i)[0],
                                       xMin,
                                       xMax,
                                       0,
                                       widthBuffer - 1);
            currYIdx = getValueInRange(ptsCoord.col(i)[1],
                                       yMin,
                                       yMax,
                                       0,
                                       heightBuffer - 1);
//            std::cout << "xIdx = " << currXIdx << " & yIdx = " << currYIdx << std::endl;
            if (isPointCloser(ptsCoord(2, i),
                              zBuffer(currXIdx, currYIdx))) {
                // save new z depth in the buffer
                zBuffer(currXIdx, currYIdx) = ptsCoord(2, i);
                // save the corresponding idx for the point in the idxBuffer
                if (idxBuffer(currXIdx, currYIdx) == -1)
                    numNearestPts++;
                idxBuffer(currXIdx, currYIdx) = i;

            }
        }
        numNearestPts--;

    }

    void getNearestPointsToCamera(Eigen::MatrixXd &projectedPts,
                                  Eigen::MatrixXd &out_nearestProjectedPts) {
        Eigen::MatrixXd zBuffer, idxBuffer;
        int numNearestPts = 0;
        createZBuffer(projectedPts,
                      zBuffer,
                      idxBuffer,
                      numNearestPts);
        out_nearestProjectedPts = Eigen::MatrixXd::Zero(3, numNearestPts);
        int colIdx = 0;

        for (int i = 0; i < idxBuffer.rows(); i++) {
            for (int j = 0; j < idxBuffer.cols(); j++) {
                if (idxBuffer(i, j) != -1) {
                    out_nearestProjectedPts.col(colIdx) = projectedPts.col(idxBuffer(i, j));
                    colIdx++;
                }
            }
        }

    }

    void mergePointClouds(PointCloud &pc1, PointCloud &pc2, Eigen::MatrixXd &out_pointSet) {
        // Author: Karina Mady

        Eigen::MatrixXd pc1_pointSet, pc2_pointSet;
        pc1.getPoints(pc1_pointSet);
        pc2.getPoints(pc2_pointSet);

        out_pointSet = Eigen::MatrixXd::Zero(3, pc1.m_numPoints + pc2.m_numPoints);
        out_pointSet << pc1_pointSet, pc2_pointSet;

        std::cout << "Merged pc1 (" << pc1.m_numPoints <<
        ") with pc2(" << pc2.m_numPoints << ")\n";
    }

    void mergePointCloudsNoDuplicates(PointCloud &pc, Eigen::MatrixXd &points_all_scans) {
        // Author: Karina Mady
        Eigen::MatrixXd currentPointSet;
        pc.getPoints(currentPointSet);
        std::cout << "The current scan matrix has " << currentPointSet.cols() << " points" << std::endl;
        std::cout << "The reconstruction matrix before this merge has "
        << points_all_scans.cols() << " points" << std::endl;

        Eigen::MatrixXd unique_m_vertices(3, currentPointSet.cols());
        int pIdx = 0;
        double threshold = 0.0001;
        int numberDuplicates = 0;

        //take every point in the current scan matrix
        for (int j = 0; j < currentPointSet.cols(); j++) {
            bool newPoint = true;
            //take every point in the reconstruction matrix
            for (int i = 0; i < points_all_scans.cols(); i++) {
                //and match them to see if there are duplicates
//              // compute the euclidean distance between the 2 vectors
                double distVect = (currentPointSet.col(j) - points_all_scans.col(i)).norm();
                // is the distance close to 0? if yes, they are duplicates
                if (distVect < threshold) {
                    //this point already exists
                    newPoint = false;
                    numberDuplicates++;
                }
            }
            //if the point has not been found, it's new, so add it as unique
            if (newPoint) {
                //save the unique points into a temporary matrix to be used later in merge
                unique_m_vertices.col(pIdx) = currentPointSet.col(j);
                pIdx++;
            }
        }
        pIdx--;

        //temporary matrix to store the unique points between the reconstruction matrix and current scan matrix
        Eigen::MatrixXd temporaryMat(3, points_all_scans.cols() + pIdx);
        temporaryMat << points_all_scans, unique_m_vertices.block(0, 0, 3, pIdx);
        points_all_scans = temporaryMat;
        std::cout << "Found " << numberDuplicates << " duplicates" << std::endl;
        std::cout << "The reconstruction matrix after merge has "
        << points_all_scans.cols() << " points" << std::endl;
    }


} //namespace nvp





