//
// Created by Maria Ruxandra Robu on 12/05/2016.
//

#ifndef NEXTVIEWPLANNING_CONVERTER_H
#define NEXTVIEWPLANNING_CONVERTER_H

#include "defs.h"
#include <Eigen/Dense>

namespace nvp{
    Eigen::Vector3d convertOMVecToEIGENVec (OpenMesh::Vec3d omCoord);

    OpenMesh::Vec3d convertEIGENVecToOMVec (Eigen::Vector3d eCoord);

    double deg2rad(double degrees);

} //namespace nvp

#endif //NEXTVIEWPLANNING_CONVERTER_H
