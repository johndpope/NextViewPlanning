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


//    bool isZBiggerThan(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2) {
//        return (vec1[2] > vec2[2]);
//    }
//
//    void swapVectors(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2) {
////        std::cout << vec1 << std::endl << vec2 << std::endl;
//        Eigen::Vector3d aux = vec1;
//        vec1 = vec2;
//        vec2 = aux;
////        std::cout << vec1 << std::endl << vec2 << std::endl;
//    }


//    Eigen::MatrixXd sortMatrixByZ(Eigen::MatrixXd& coordMat) {
////        std::cout << "Before:\n";
////        std::cout << coordMat << std::endl;
//
//        if (coordMat.size() == 0) {
//            std::cerr << "ERROR! Utilities.cpp: Empty matrix \n";
//            throw ERROR;
//        }
//        std::cout << "Matrix size = " << coordMat.size() << std::endl;
//        double numPts = coordMat.cols();
//        double numCoord = coordMat.rows();
//
//        if (numCoord != 3) {
//            std::cerr << "ERROR! Utilities.cpp: Each column should have 3 coord: x,y,z \n";
//            throw ERROR;
//        }
//
//
//
//
////        bool swapped = true;
////        while (swapped) {
////            swapped = false;
////            for (int i = 1; i < numPts; i++) {
////                Eigen::Vector3d prevPoint = coordMat.col(i - 1);
////                Eigen::Vector3d curPoint = coordMat.col(i);
////
////                if (isZBiggerThan(prevPoint, curPoint)) {
////                    swapVectors(prevPoint, curPoint);
////                    coordMat.col(i - 1) = prevPoint;
////                    coordMat.col(i) = curPoint;
////                    swapped = true;
////                }
////            }
////            numPts --;
////        }
////        std::cout << "After:\n";
////        std::cout << coordMat << std::endl;
//        return coordMat;
//    }


} //namespace nvp





