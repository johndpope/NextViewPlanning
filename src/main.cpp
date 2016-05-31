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

void testCameraRotation();


int main() {
    //testCameraRotation();
    //return SUCCESS;
    //testFramework();
    //testFrameworkForMultipleScans();
    //testNormalComputation();
// TODO: add full framework that sets a path -> with stopping condition
    //****************************************************************
    int numberMesh = 2;
//    std::cout << "Enter <1> for the armadillo, "
//            "<2> for the bunny or "
//            "<3> for the dragon mesh \n";
//    std::cin >> numberMesh;
//
//    if (numberMesh != 1 && numberMesh != 2 && numberMesh != 3) {
//        std::cout << "Please enter <1> or <2> or <3> \n";
//        std::cin >> numberMesh;
//    }

    // read bunny by default - fewer points
    PointCloud originalPCD(MODEL_2_FILENAME);
//    if (numberMesh == 1) {
//        // read armadillo mesh
//        originalPCD = PointCloud(MODEL_1_FILENAME);
//    }
//    else if (numberMesh == 3) {
//        // read dragon mesh
//        originalPCD = PointCloud(MODEL_3_FILENAME);
//    }

    std::cout << originalPCD.m_numPoints << " points read" << std::endl;

    //****************************************************************
    // Generate k given scan positions
    int kPositions = 2;
    std::vector<Camera> kViewVector;
    Eigen::VectorXd degreesYRotation(kPositions);
    // NOTE: it works starting with two or more viewpoints
    degreesYRotation << 10, 40;
    getCameraVecFromDegrees(originalPCD, degreesYRotation, kViewVector);

    //****************************************************************
    // Compute the Next Best View
    Camera kplus1View = computeNBV(originalPCD,
                                   kViewVector);

    std::vector<Camera> kplus1ViewVector = getKplus1ViewVector(kViewVector,
                                                               kplus1View);

    // ******************************************************
    // Evaluate current NBV against the original mesh
    int zbufferSideSize = 200;

    std::cout << "Eval with GT - Compute score for chosen NBV with "
    << kplus1View.getRotationYDegrees() << " degrees\n";
    double scoreGT = evaluateNBV(kplus1ViewVector,
                                 originalPCD,
                                 zbufferSideSize);
    printScoreToConsole(scoreGT);


    // ******************************************************
    // Clean up - Estimate PCD from k+1 views and write it
    std::cout << "Estimate PCD from k+1 views...\n";
    Eigen::MatrixXd kplus1PCDEstimation;
    zbufferSideSize = 200;
    getEstimatedReconstructionFromKViews(originalPCD,
                                         kplus1ViewVector,
                                         kplus1PCDEstimation,
                                         zbufferSideSize);

    PointCloud estimatedPCD(kplus1PCDEstimation);
    std::string filenameEstimPCD = "../output/estimatedPCD.ply";
    estimatedPCD.write(filenameEstimPCD);

    std::cout << "Done!\n";

    return SUCCESS;
}

void testNormalComputation() {
    PointCloud originalPCD("../output/estimated_scan_1.ply");
    //read point cloud in
    std::cout << originalPCD.m_numPoints << " point read" << std::endl;
    originalPCD.setNormals();

    originalPCD.write("../output/estimated_scan_1_withNormals.ply");
    std::cout << "Done!\n";
}

void testFrameworkForMultipleScans() {
    int numberMesh;
    std::cout << "Enter <1> for the armadillo or <2> for the bunny mesh: \n";
    std::cin >> numberMesh;

    if (numberMesh != 1 && numberMesh != 2) {
        std::cout << "Please enter <1> or <2> \n";
        std::cin >> numberMesh;
    }

    // read bunny by default - fewer points
    PointCloud originalPCD(MODEL_3_FILENAME);
    if (numberMesh == 1) {
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

void testFramework() {

    PointCloud originalPCD(MODEL_2_FILENAME);

//    std::cout << "Before viewp: \n" << myPointSetBefore << std::endl;
    std::cout << originalPCD.m_numPoints << " points read\n";

    std::cout << "Process 3d point cloud..." << std::endl;
    // set the rotation and translation for the extrinsics of the camera
    // change the Y rotation to move along the horizontal axis of the mesh
    double rotationY_deg = 180.0;

    Camera currentCamera = getCameraFromDegrees(originalPCD, rotationY_deg);

    // use this to obtain all of the projected points, irrespective of their depth
    // pc.computeProjectedCoordinates(currentCamera);

    // use this to compute the nearest points as seen from the camera (zBuffer)
    originalPCD.computeNearestProjectedPts(currentCamera);

    // get the cartesian coordinates of the projected points
    // Eigen::MatrixXd cartesianCoord;
    // pc.getCartesianCoordinates(cartesianCoord);

    // convert the projected points back to the world coordinate frame
    originalPCD.computeWorldCoordinates(currentCamera);

    originalPCD.write(MODEL_2_OUTPUT_FILENAME);
    std::cout << "Magic!" << std::endl;

}

void testCameraRotation() {
    PointCloud myPCD(MODEL_1_FILENAME);

    std::vector<Camera> camVector;
    Eigen::VectorXd degreesYRotation(4);
    degreesYRotation << 0, 90, 180, 270;
    getCameraVecFromDegrees(myPCD, degreesYRotation, camVector);

    writePCDAndCamera(myPCD, camVector);
}