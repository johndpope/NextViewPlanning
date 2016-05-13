//
// Created by Maria Ruxandra Robu on 13/05/2016.
//

#ifndef NEXTVIEWPLANNING_UTILITIES_H
#define NEXTVIEWPLANNING_UTILITIES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace nvp{
    Eigen::Matrix4d createTransformationMatrix(double rotX, double rotY, double rotZ);


}//namespace nvp


#endif //NEXTVIEWPLANNING_UTILITIES_H
