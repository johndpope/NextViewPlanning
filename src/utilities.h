//
// Created by Maria Ruxandra Robu on 13/05/2016.
//

#ifndef NEXTVIEWPLANNING_UTILITIES_H
#define NEXTVIEWPLANNING_UTILITIES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace nvp{
    Eigen::Matrix4d createRotationMatrix(double rotRadiansX,
                                         double rotRadiansY,
                                         double rotRadiansZ);


}//namespace nvp


#endif //NEXTVIEWPLANNING_UTILITIES_H
