#include <iostream>
#include "PointCloud.h"
#include "defs.h"
#include "utilities.h"
#include "Camera.h"
#include <Eigen/Dense>
#include <vector>
#include "NextBestView.h"


using namespace nvp;
void testFramework();
void testFrameworkForMultipleScans();
void testNormalComputation();



int main() {
    //testFramework();
    //testFrameworkForMultipleScans();
    testNormalComputation();

    //****************************************************************
//    int numberMesh;
//    std::cout << "Enter <1> for the armadillo or <2> for the bunny mesh: \n";
//    std::cin >> numberMesh;
//
//    if (numberMesh != 1 && numberMesh != 2) {
//        std::cout << "Please enter <1> or <2> \n";
//        std::cin >> numberMesh;
//    }
//
//    std::cout << "Reading 3D model..." << std::endl;
//    // read bunny by default - fewer points
//    PointCloud originalPCD(MODEL_3_FILENAME);
//    if(numberMesh == 1){
//        // read armadillo mesh
//        originalPCD = PointCloud(MODEL_1_FILENAME);
//    }
//    //read point cloud in
//    std::cout << originalPCD.m_numPoints << " point read" << std::endl;
//    originalPCD.setNormals();
//
//    //vector of cameras
//    std::vector<Camera> scans;
//    //generate the k given scans
//
//
//    //compute the NBV
//
//
//    //evaluate current NBV against the original mesh
//
//
////    Eigen::MatrixXd mergedScans;
////    getEstimatedReconstruction(scans, mergedScans);
////
////    std::string filenameEstimPCD = "../output/estimatedPCD.ply";
////    PointCloud estimatedPCD(mergedScans);
////    estimatedPCD.write(filenameEstimPCD);
////
////    std::cout << "Quality check..." << std::endl;
////    compareOriginalWithReconstruction(originalPCD, estimatedPCD);
//
//    std::cout << "Writing 3D model to output folder..." << std::endl;
//    if(numberMesh == 1){
//        // write armadillo mesh
//        originalPCD.write(MODEL_1_OUTPUT_FILENAME);
//    }
//    else
//    {
//        originalPCD.write(MODEL_3_OUTPUT_FILENAME);
//    }
//
//    std::cout << "Done!\n";

    return SUCCESS;
}

void testNormalComputation(){
    PointCloud originalPCD("../output/estimated_scan_1.ply");
    //read point cloud in
    std::cout << originalPCD.m_numPoints << " point read" << std::endl;
    originalPCD.setNormals();

    std::cout << "Writing 3D model to output folder..." << std::endl;
    originalPCD.write("../output/estimated_scan_1_withNormals.ply");
    std::cout << "Done!\n";
}

void testFrameworkForMultipleScans(){
    int numberMesh;
    std::cout << "Enter <1> for the armadillo or <2> for the bunny mesh: \n";
    std::cin >> numberMesh;

    if (numberMesh != 1 && numberMesh != 2) {
        std::cout << "Please enter <1> or <2> \n";
        std::cin >> numberMesh;
    }

    std::cout << "Reading 3D model..." << std::endl;
    // read bunny by default - fewer points
    PointCloud originalPCD(MODEL_3_FILENAME);
    if(numberMesh == 1){
        // read armadillo mesh
        originalPCD = PointCloud(MODEL_1_FILENAME);
    }
    //read point cloud in
    std::cout << originalPCD.m_numPoints << " point read" << std::endl;

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
}

void testFramework(){
    std::cout << "Reading 3D model from models folder..." << std::endl;

    PointCloud originalPCD(MODEL_1_FILENAME);

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
    originalPCD.write(MODEL_1_OUTPUT_FILENAME);
    std::cout << "Magic!" << std::endl;

}