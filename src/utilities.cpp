//
// Created by Maria Ruxandra Robu on 13/05/2016.
//
#include <iostream>
#include "defs.h"
#include "utilities.h"
#include <vector>

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


    double getValueInRange(double value,
                           double currentMin,
                           double currentMax,
                           double rangeMin,
                           double rangeMax) {
        // This function brings the *value* which is in the range [currentMin, currentMax]
        // into [rangeMin,rangeMax]

        return rangeMin + ((rangeMax - rangeMin) * (value - currentMin)) / (currentMax - currentMin);
    }

    bool isPointCloser(double newZ, double currentZ) {
        return newZ < currentZ;
    }

    void createZBuffer(Eigen::MatrixXd &ptsCoord,
                       Eigen::MatrixXd &zBuffer,
                       Eigen::MatrixXd &idxBuffer,
                       long& numNearestPts) {
        numNearestPts = 0;
        // if steps = 1, it might lead to stack overflow
        // => we need better memory management
        // => the use sparse matrices for the zBufffer and the idxBuffer
        double steps = 500.0;
        double xMin = ptsCoord.row(0).minCoeff();
        double xMax = ptsCoord.row(0).maxCoeff();
        double yMin = ptsCoord.row(1).minCoeff();
        double yMax = ptsCoord.row(1).maxCoeff();
        double zMin = ptsCoord.row(2).minCoeff();
        double zMax = ptsCoord.row(2).maxCoeff();
        long widthBuffer = std::floor(std::abs(xMax - xMin) / steps);
        long heightBuffer = std::floor(std::abs(yMax - yMin) / steps);

        zBuffer = zMax * Eigen::MatrixXd::Ones(widthBuffer, heightBuffer);
        idxBuffer = -1 * Eigen::MatrixXd::Ones(widthBuffer, heightBuffer);

        double currXIdx, currYIdx;
        for (int i = 0; i < ptsCoord.cols(); ++i)
        {
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


            std::cout<<"crashes inside isPointCloser -> utilities.cpp : createZBuffer() "<<std::endl;
            if(isPointCloser(ptsCoord(2,i),
                             zBuffer(currXIdx,currYIdx)))
            {

                std::cout<<"this never gets printed"<<std::endl;
                // save new z depth in the buffer
                zBuffer(currXIdx,currYIdx) = ptsCoord(2,i);
                // save the corresponding idx for the point in the idxBuffer
                idxBuffer(currXIdx,currYIdx) = i;
                numNearestPts++ ;
            }
        }
    }

    void getNearestPointsToCamera(Eigen::MatrixXd &projectedPts,
                                  Eigen::MatrixXd &out_nearestProjectedPts) {
        Eigen::MatrixXd zBuffer, idxBuffer;
        long numNearestPts = 0;
        createZBuffer(projectedPts,
                      zBuffer,
                      idxBuffer,
                      numNearestPts);

        out_nearestProjectedPts = Eigen::MatrixXd::Zero(3,numNearestPts);
        long colIdx = 0;

        for( int i = 0; i < idxBuffer.rows(); ++i)
        {
            for( int j = 0; j < idxBuffer.cols(); ++j)
            {
                if (idxBuffer(i,j) != -1)
                {
                    out_nearestProjectedPts.col(colIdx) = projectedPts.col(idxBuffer(i,j));
                    colIdx++;
                }
            }
        }

    }

//    bool isZBiggerThan(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2)
//    {
//        return (vec1[2] > vec2[2]);
//    }

} //namespace nvp





