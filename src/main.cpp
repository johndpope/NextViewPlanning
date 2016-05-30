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

    std::cout << "Reading 3D model..." << std::endl;
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
//    originalPCD.setNormals();

    //****************************************************************
    // Generate k given scan positions
    int kPositions = 3;
    std::vector<Camera> kViewVector;
    Eigen::VectorXd degreesYRotation(kPositions);
    // it works starting with one or more viewpoints
    degreesYRotation << 10, 40, 120;
    getCameraVecFromDegrees(originalPCD, degreesYRotation, kViewVector);

    // ******************************************************
    // Get candidate views for the NBV
    Eigen::VectorXd candidateYRotDegrees;
    getCandidateViewsDegrees(kViewVector,
                             candidateYRotDegrees);
    int noCandidates = int(candidateYRotDegrees.size());

    // ******************************************************
    // Compute scores for each new view (no ground truth) =
    // the amount of new points brought by each candidate scan
    // NOTE: we use a lower resolution of each scan to make the computation faster
    // TODO: add score based on point quality as well
    Eigen::VectorXd scoresCandidateViews;
    evaluateEachCandidateView(originalPCD,
                              kViewVector,
                              candidateYRotDegrees,
                              scoresCandidateViews);

    // get maximum score and save that as the NBV
    Eigen::VectorXd::Index maxRow, maxCol;
    int maxScoreNBV = int(scoresCandidateViews.maxCoeff(&maxRow, &maxCol));
    std::cout << "Found maximum score of " << scoresCandidateViews[maxRow] <<
    " for " << candidateYRotDegrees[maxRow] << " degrees\n";
    Camera kplus1View = getCameraFromDegrees(originalPCD, candidateYRotDegrees[maxRow]);
    std::vector<Camera> kplus1ViewVector = getKplus1ViewVector(kViewVector,
                                                               kplus1View);

    // ******************************************************
    // Evaluate current NBV against the original mesh
    int zbufferSideSize = 150;

    std::cout << "Eval with GT - Compute score for chosen NBV with "
    << candidateYRotDegrees[maxRow] << " degrees\n";
    double scoreGT = evaluateNBV(kplus1ViewVector,
                                 originalPCD,
                                 zbufferSideSize);
    printScoreToConsole(scoreGT);

    // ******************************************************
    // ********************** OPTIONAL **********************
    // Check with the ground truth if we actually chose the best candidate view
    Eigen::VectorXd scoresGTVec(noCandidates);
    for (int i = 0; i < noCandidates; i++) {
        std::cout << "Eval with GT - Compute score for candidate view #" << i + 1 << ": ";
        Camera kplus1View_temp = getCameraFromDegrees(originalPCD,
                                                      candidateYRotDegrees[i]);

        // create a temporary vector of k+1 views for each candidate position
        std::vector<Camera> kplus1ViewVector_temp = getKplus1ViewVector(kViewVector,
                                                                        kplus1View_temp);
        scoresGTVec[i] = evaluateNBV(kplus1ViewVector_temp,
                                     originalPCD,
                                     zbufferSideSize);
        std::cout << scoresGTVec[i] << std::endl;
    }
    Eigen::VectorXd::Index maxRowGT, maxColGT;
    int maxScoreGT = int(scoresGTVec.maxCoeff(&maxRowGT, &maxColGT));
    std::cout << "Eval with GT - Found maximum score of " << scoresGTVec[maxRowGT] <<
    " for " << candidateYRotDegrees[maxRowGT] << " degrees\n";

    // ******************************************************
    // Clean up - Estimate PCD from k+1 views and write it
    std::cout << "Estimate PCD from k+1 views...\n";
    Eigen::MatrixXd kplus1PCDEstimation;
    zbufferSideSize = 150;
    getEstimatedReconstructionFromKViews(originalPCD,
                                         kplus1ViewVector,
                                         kplus1PCDEstimation,
                                         zbufferSideSize);

    std::cout << "Writing 3D model to output folder..." << std::endl;
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

    std::cout << "Writing 3D model to output folder..." << std::endl;
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

    std::cout << "Reading 3D model..." << std::endl;
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
    std::cout << "Reading 3D model from models folder..." << std::endl;

    PointCloud originalPCD(MODEL_2_FILENAME);

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
    originalPCD.write(MODEL_2_OUTPUT_FILENAME);
    std::cout << "Magic!" << std::endl;

}