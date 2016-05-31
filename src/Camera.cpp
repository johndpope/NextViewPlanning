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
        m_degreesXRot = rotDegreesX;
        m_degreesYRot = rotDegreesY;
        m_degreesZRot = rotDegreesZ;
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

    double Camera::getRotationYDegrees() {
        return m_degreesYRot;
    }

    Camera &Camera::operator=(const Camera &pSrc) {
// check for self - assignment
        if (this == &pSrc)
            return *this;

        m_degreesXRot = pSrc.m_degreesXRot;
        m_degreesYRot = pSrc.m_degreesYRot;
        m_degreesZRot = pSrc.m_degreesZRot;
        m_cameraTransf = pSrc.m_cameraTransf;
        m_intrinsics = pSrc.m_intrinsics;
        m_extrinsics = pSrc.m_extrinsics;
        m_focalLengthxy = pSrc.m_focalLengthxy;
        m_offsetXY = pSrc.m_offsetXY;

        // return the existing object
        return *this;    }

    Camera::Camera(const Camera &pSrc) {
        m_degreesXRot = pSrc.m_degreesXRot;
        m_degreesYRot = pSrc.m_degreesYRot;
        m_degreesZRot = pSrc.m_degreesZRot;
        m_cameraTransf = pSrc.m_cameraTransf;
        m_intrinsics = pSrc.m_intrinsics;
        m_extrinsics = pSrc.m_extrinsics;
        m_focalLengthxy = pSrc.m_focalLengthxy;
        m_offsetXY = pSrc.m_offsetXY;
    }

    Eigen::Vector3d Camera::getCameraOrientation() {
        // So if the "in" axis is the z-axis, for instance,
        // then the vector pointing in the direction
        // the camera is pointing

        Eigen::Matrix3d rotationMat = m_extrinsics.block(0,0,3,3);
        Eigen::Vector3d zAxis(3);
        zAxis << 0,0,1;
        Eigen::Vector3d camDirection = rotationMat.inverse() * zAxis;

//        std::cout << "Camera direction: " << camDirection.transpose()
//        << " for a rotation of " << m_degreesYRot << std::endl;

        return camDirection;
    }

    Eigen::Vector3d Camera::getCameraPosition() {
        Eigen::Matrix3d rotationMat = m_extrinsics.block(0,0,3,3);
        Eigen::Vector3d translation = m_extrinsics.block(0,3,3,1);

        Eigen::Vector3d camPos = - rotationMat.inverse() * translation;

//        std::cout << "Camera position: " << camPos.transpose()
//        << " for a rotation of " << m_degreesYRot << std::endl;
        return camPos;
    }

} //namespace nvp






