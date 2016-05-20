//
// Created by Maria Ruxandra Robu on 13/05/2016.
//
#include "iostream"
#include "utilities.h"

namespace nvp {

    Eigen::Matrix4d createRotationMatrix(double rotRadiansX,
                                         double rotRadiansY,
                                         double rotRadiansZ){

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



} //namespace nvp





