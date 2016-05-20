//
// Created by Maria Ruxandra Robu on 19/05/2016.
//

#include "Camera.h"
#include "utilities.h"
#include "converter.h"
namespace nvp {
    Camera::Camera(Eigen::Matrix4d extrinsics) {
        setCameraIntrinsics(m_focalLengthxy,m_offsetXY);
        m_extrinsics = extrinsics;
        m_cameraTransf = m_intrinsics * m_extrinsics;
    }

    Camera::Camera(double rotDegreesX,
                   double rotDegreesY,
                   double rotDegreesZ,
                   double translX,
                   double translY,
                   double translZ,
                   double focalLength,
                   double offsetXY) {
        m_focalLengthxy = focalLength;
        m_offsetXY = offsetXY;
        setCameraIntrinsics(m_focalLengthxy,m_offsetXY);

        m_extrinsics = createRotationMatrix(deg2rad(rotDegreesX),
                                            deg2rad(rotDegreesY),
                                            deg2rad(rotDegreesZ));
        // set translations
        m_extrinsics(0, 3) = translX;
        m_extrinsics(1, 3) = translY;
        m_extrinsics(2, 3) = translZ;

        m_cameraTransf = m_intrinsics * m_extrinsics;

    }

    void Camera::getCameraTransform(Eigen::Matrix4d& camTransf) {
        camTransf = m_cameraTransf;
    }

    void Camera::setCameraIntrinsics(double focalLength,
                                            double offsets) {
        m_intrinsics = Eigen::MatrixXd::Identity(4,4);
//    intrinsicMat(2,2) = 1;
        m_intrinsics(0,2) = offsets; // offset x
        m_intrinsics(1,2) = offsets; // offset y
        m_intrinsics(0,0) = focalLength; // focal length x
        m_intrinsics(1,1) = focalLength; // focal length y
    }


} //namespace nvp






