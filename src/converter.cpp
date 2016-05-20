//
// Created by Maria Ruxandra Robu on 12/05/2016.
//

#include "converter.h"

namespace nvp{
    Eigen::Vector3d convertOMVecToEIGENVec (OpenMesh::Vec3d omCoord)
    {
        Eigen::Vector3d eCoord(omCoord[0],omCoord[1],omCoord[2]);
        return eCoord;
    }

    OpenMesh::Vec3d convertEIGENVecToOMVec (Eigen::Vector3d eCoord)
    {
        OpenMesh::Vec3d omCoord(eCoord[0],eCoord[1],eCoord[2]);
        return omCoord;
    }

    double deg2rad(double degrees) {
        return (degrees * M_PI/180.0);
    }


} //namespace nvp


