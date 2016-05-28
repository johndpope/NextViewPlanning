//
// Created by Maria Ruxandra Robu on 12/05/2016.
//

#ifndef NEXTVIEWPLANNING_CONVERTER_H
#define NEXTVIEWPLANNING_CONVERTER_H

#include "defs.h"
#include "ANN/ANN.h"
#include <Eigen/Dense>
#include <stdlib.h>     /* malloc, free, rand */


namespace nvp{
    Eigen::Vector3d convertOMVecToEIGENVec (OpenMesh::Vec3d omCoord);

    OpenMesh::Vec3d convertEIGENVecToOMVec (Eigen::Vector3d eCoord);

    double deg2rad(double degrees);

    Eigen::VectorXd convertANNpointToEigenVec (ANNpoint& in_annP,
                                               int dim);

    ANNpoint convertEigenVecToANNpoint (Eigen::VectorXd& in_eigP);

    Eigen::MatrixXd convertANNarrayToEigenMat (ANNpointArray& in_annArray,
                                               int dim,
                                               int nPts);

    ANNpointArray convertEigenMatToANNarray (Eigen::MatrixXd& in_eigArray);

} //namespace nvp

#endif //NEXTVIEWPLANNING_CONVERTER_H
