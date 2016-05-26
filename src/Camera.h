//
// Created by Maria Ruxandra Robu on 19/05/2016.
//

#ifndef NEXTVIEWPLANNING_CAMERA_H
#define NEXTVIEWPLANNING_CAMERA_H
#include <Eigen/Dense>
#include "defs.h"

namespace nvp {
    class Camera {
    public:
        Camera(Eigen::Matrix4d extrinsics);
        Camera(double rotDegreesX = 0.0,
               double rotDegreesY = 0.0,
               double rotDegreesZ = 0.0,
               double translX = 0.0,
               double translY = 0.0,
               double translZ = 0.0,
               double focalLength = FOCAL_LENGTH,
               double offsetXY = IMAGEPLANE_SIZE/2);

        void getCameraTransform(Eigen::Matrix4d& camTransf);

    private:
        void setCameraIntrinsics(double focalLength,
        double offsets);
        Eigen::Matrix4d m_cameraTransf;
        Eigen::Matrix4d m_intrinsics;
        Eigen::Matrix4d m_extrinsics;
        double m_focalLengthxy = FOCAL_LENGTH;
        double m_offsetXY = IMAGEPLANE_SIZE/2;


    };
} //namespace nvp


#endif //NEXTVIEWPLANNING_CAMERA_H
