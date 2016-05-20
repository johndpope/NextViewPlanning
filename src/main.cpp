#include <iostream>
#include "PointCloud.h"
#include "defs.h"
#include "utilities.h"
#include "Camera.h"
#include <Eigen/Dense>
#include <Eigen/LU>


using namespace nvp;


int main() {
    std::cout << "Reading 3D model from models folder..." << std::endl;

    PointCloud pc(MODEL_1_FILENAME);

    // get the point set - dimensions: 3 x numPts
//    Eigen::MatrixXd myPointSetBefore;
//    pc.getPoints(myPointSetBefore);
//    std::cout << "Before viewp: \n" << myPointSetBefore << std::endl;
    std::cout << pc.m_numPoints << " points read\n";

    std::cout << "Process 3d point cloud..." << std::endl;
    // set the rotation and translation for the extrinsics of the camera
    // change the Y rotation to move along the horizontal axis of the mesh
    double rotationX_deg = 0.0, rotationY_deg = 180.0, rotationZ_deg = 0.0;
    double translationX, translationY, translationZ;

    // center the camera wrt to the point cloud
    pc.getCenterXY(translationX, translationY);
    translationZ = pc.computeRadiusFromCentroid();

    // create a new camera object
    Camera currentCamera(rotationX_deg,
                         rotationY_deg,
                         rotationZ_deg,
                         translationX,
                         translationY,
                         translationZ);

    // use this to obtain all of the projected points, irrespective of their depth
    // pc.computeProjectedCoordinates(currentCamera);

    // clip points by setting a threshold at half the distance between minZ and maxZ
    // this is not really what we want, since we don't get all of the points
    // that would be seen first from the camera
    // pc.clipPointsByZ();

    // use this to compute the nearest points as seen from the camera
    // a discretization step of 500 is used for now
    pc.computeNearestProjectedPts(currentCamera);

    // get the cartesian coordinates of the projected points
    // Eigen::MatrixXd cartesianCoord;
    // pc.getCartesianCoordinates(cartesianCoord);


    // convert the projected points back to the world coordinate frame
    // pc.computeWorldCoordinates(currentCamera);

    std::cout << "Writing 3D model to output folder..." << std::endl;
    pc.write(MODEL_1_OUTPUT_FILENAME);
    std::cout << "Magic!" << std::endl;

    return SUCCESS;
}