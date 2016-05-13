//
// Created by Maria Ruxandra Robu on 13/05/2016.
//
#include "iostream"
#include "utilities.h"

namespace nvp {

    Eigen::Matrix4d createTransformationMatrix(double rotX, double rotY, double rotZ) {
        // for now, the transformation matrix is just a rotation
        Eigen::Matrix4d transfMat(Eigen::Matrix4d::Identity());


        Eigen::Affine3d rx =
                Eigen::Affine3d(Eigen::AngleAxisd(rotX, Eigen::Vector3d(1, 0, 0)));
        Eigen::Affine3d ry =
                Eigen::Affine3d(Eigen::AngleAxisd(rotY, Eigen::Vector3d(0, 1, 0)));
        Eigen::Affine3d rz =
                Eigen::Affine3d(Eigen::AngleAxisd(rotZ, Eigen::Vector3d(0, 0, 1)));
        Eigen::Affine3d rotMat = rz * ry * rx;

        transfMat = rotMat.matrix();

//        std::cout << "rotationMat: \n" << rotMat.matrix() << std::endl;
        return transfMat;
    }


} //namespace nvp





