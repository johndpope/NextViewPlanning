#include <iostream>
#include "PointCloud.h"
#include "defs.h"
#include "utilities.h"
#include <Eigen/Dense>


using namespace nvp;


int main() {
    std::cout << "Reading 3D model from models folder..." << std::endl;

    PointCloud pc(MODEL_1_FILENAME);

    // get the point set - dimensions: 3 x numPts
    Eigen::MatrixXd myPointSet;
    pc.getPoints(myPointSet);
    std::cout << pc.m_numPoints << " points read\n";
    // std::cout << myPointSet.cols() << " points read\n";

    std::cout << "Process 3d point cloud..." << std::endl;
    // set the modified point set in the same point cloud - dimensions: 3 x numPts
//    Eigen::MatrixXd newPointSet(myPointSet);
//    pc.setPoints(newPointSet);

    // apply a transformation to the whole point cloud
    double x_rad = 0.0 * M_PI/180, y_rad = 0.0 * M_PI/180, z_rad = 45.00 * M_PI/180;
    Eigen::Matrix4d transformationMat = createTransformationMatrix(x_rad, y_rad, z_rad);
    pc.applyTransformation(transformationMat);

    std::cout << "Writing 3D model to output folder..." << std::endl;
    pc.write(MODEL_1_OUTPUT_FILENAME);
    std::cout << "Magic!" << std::endl;

    return SUCCESS;
}