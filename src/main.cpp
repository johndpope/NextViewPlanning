#include <iostream>
#include "PointCloud.h"
#include "defs.h"
#include "utilities.h"
#include "Camera.h"
#include <Eigen/Dense>
#include "NextBestView.h"


using namespace nvp;
void testFramework();


int main() {
    //testFramework();

    std::cout << "Reading 3D model..." << std::endl;
    //read point cloud in
    PointCloud originalPCD(MODEL_1_FILENAME);

    //vector of cameras
    std::vector<Camera> scans;
    //this generates a number of scans based on degrees input by the user
    //and writes the new scans to separate files
    generateScansFromDegrees(scans, originalPCD);
    std::cout << "Scans created : " << scans.size() << std::endl;


    Eigen::MatrixXd mergedScans;
    getEstimatedReconstruction(scans, mergedScans);

    std::string filenameEstimPCD = "../output/estimatedPCD.ply";
    PointCloud estimatedPCD(mergedScans);
    estimatedPCD.write(filenameEstimPCD);

    std::cout << "Quality check..." << std::endl;
    compareOriginalWithReconstruction(originalPCD, estimatedPCD);

    std::cout << "Magic!";

    return SUCCESS;
}

void testFramework(){
    std::cout << "Reading 3D model from models folder..." << std::endl;

    PointCloud originalPCD(MODEL_3_FILENAME);

//    std::cout << "Before viewp: \n" << myPointSetBefore << std::endl;
    std::cout << originalPCD.m_numPoints << " points read\n";

    std::cout << "Process 3d point cloud..." << std::endl;
    // set the rotation and translation for the extrinsics of the camera
    // change the Y rotation to move along the horizontal axis of the mesh
    double rotationX_deg = 0.0, rotationY_deg = 90.0, rotationZ_deg = 0.0;
    double translationX, translationY, translationZ;

    // center the camera wrt to the point cloud
    originalPCD.getCenterXY(translationX, translationY);
    translationZ = originalPCD.computeRadiusFromCentroid();

    // create a new camera object
    Camera currentCamera(rotationX_deg,
                         rotationY_deg,
                         rotationZ_deg,
                         translationX,
                         translationY,
                         translationZ);

    // use this to obtain all of the projected points, irrespective of their depth
    // pc.computeProjectedCoordinates(currentCamera);

    // use this to compute the nearest points as seen from the camera (zBuffer)
    originalPCD.computeNearestProjectedPts(currentCamera);

    // get the cartesian coordinates of the projected points
    // Eigen::MatrixXd cartesianCoord;
    // pc.getCartesianCoordinates(cartesianCoord);

    // convert the projected points back to the world coordinate frame
    originalPCD.computeWorldCoordinates(currentCamera);

    std::cout << "Writing 3D model to output folder..." << std::endl;
    originalPCD.write(MODEL_3_OUTPUT_FILENAME);
    std::cout << "Magic!" << std::endl;

}