#include <iostream>
#include "PointCloud.h"
#include "defs.h"
#include "utilities.h"
#include "Camera.h"
#include <Eigen/Dense>
#include <Eigen/LU>


using namespace nvp;


void generateScansFromDegrees(std::vector<Camera> &scans, PointCloud &pc) {
    // set the rotation and translation for the extrinsics of the camera
    double rotationX_deg = 0.0, rotationY_deg = 0.0, rotationZ_deg = 0.0;
    double translationX, translationY, translationZ;

    //degree offset given by user
    double degrees_input = 0.0;

    std::cout << "Please enter the shift in degrees between each scan: ";
    std::cin >> degrees_input;

    if (degrees_input == 0 || degrees_input == 360) {
        std::cout << "Please enter a number > 0 and < 360";
        std::cin >> degrees_input;
    }

    //calculate the number of scans needed to cover the whole model based on input of degrees
    int num_scans = int(std::floor(360 / degrees_input));
    std::cout << "Number of scans needed: " << num_scans << std::endl;
    // center the camera wrt to the point cloud
    pc.getCenterXY(translationX, translationY);
    translationZ = pc.computeRadiusFromCentroid();

    for (auto camNo = 0; camNo < num_scans; camNo++) {
        rotationY_deg += degrees_input;
        std::cout << "Scan: " << camNo << " & Rotation y = " << rotationY_deg << std::endl;

        // transform the original point cloud by the yDegrees
        PointCloud pcTransformed(pc);

        // create a new camera object
        Camera currentCamera(rotationX_deg,
                             rotationY_deg,
                             rotationZ_deg,
                             translationX,
                             translationY,
                             translationZ);

        scans.push_back(currentCamera);

        pcTransformed.computeNearestProjectedPts(currentCamera);

        //write each scan to a different file
        std::string outputFile = "../output/scan_" +
                                 std::to_string(camNo + 1) + ".ply";
        //std::cout << "Writing file " << outputFile << std::endl;
        pcTransformed.writeCloud(outputFile);
        //std::cout << "File " << outputFile << " written" << std::endl;
    }
}


void writeReconstructedMeshToFile(std::vector<Camera> const &scans,
                                  Eigen::MatrixXd &points_all_scans,
                                  PointCloud &reconstruction_cloud) {

    std::cout << "\nStarting mesh reconstruction..." << std::endl;
    std::string reconstructedMesh = "../output/reconstruction.ply";

    for (auto camNo = 0; camNo < scans.size(); ++camNo) {
        //for every scan create a point cloud and read the points in
        std::string inputFile = "../output/scan_" +
                                std::to_string(camNo + 1) + ".ply";
        PointCloud pc(inputFile);

        Camera currentCamera(scans[camNo]);
        pc.computeWorldCoordinates(currentCamera);

        std::string outputFile = "../output/reconstructed_scan_" +
                                 std::to_string(camNo + 1) + ".ply";
        //std::cout << "Writing file " << outputFile << std::endl;
        pc.writeCloud(outputFile);

        //this function takes each point cloud and concatenates them into a single matrix
        //this can be improved to avoid duplicate point being written
        pc.mergeClouds(points_all_scans);
    }

    std::cout << "test test points_all_scans.cols(0)[0] is" << points_all_scans.col(0)[0] << std::endl;
    std::cout<<"test points_all_scans.cols() is "<<points_all_scans.cols()<<std::endl;
    reconstruction_cloud.setPoints(points_all_scans);
    //std::cout<<"reconstruction_cloud.m_numPoints is "<<reconstruction_cloud.m_numPoints<<std::endl;

    // output to a single file using the write function
    std::cout << "Finished merging the clouds" << std::endl;
    std::cout << "Writing reconstructed cloud to file ..." << std::endl;
    reconstruction_cloud.writeCloud(reconstructedMesh);
    std::cout << "File " << reconstructedMesh << " written" << std::endl;

}

//compares the vertices between ic (input cloud) and rc(reconstructed cloud)
void compareInputMeshWithReconstruction(PointCloud &ic, PointCloud &rc) {
    Eigen::MatrixXd inputCloudPoints, reconstructedCloudPoints;
    ic.getPoints(inputCloudPoints);
    rc.getPoints(reconstructedCloudPoints);

    if (ic.m_numPoints == rc.m_numPoints)
        std::cout << "Smashed it! We got all the vertices" << std::endl;
    else if (ic.m_numPoints < rc.m_numPoints)
        std::cout << "We have more vertices than we started with. Duplicates?" << std::endl;
    else if (ic.m_numPoints > rc.m_numPoints)
        std::cout << "You're missing vertices, John Snow! The scans don't capture everything" << std::endl;
    else
        std::cout << "You should change your career,Khaleesi" << std::endl;

}

int main() {

    std::cout << "Reading 3D model..." << std::endl;
    //read point cloud in
    PointCloud pc(MODEL_3_FILENAME);

    // get the point set - dimensions: 3 x numPts
    Eigen::MatrixXd myPointSetBefore;
    pc.getPoints(myPointSetBefore);

    //vector of cameras
    std::vector<Camera> scans;
    //this generates a number of scans based on degrees input by the user
    //and writes the new scans to separate files
    generateScansFromDegrees(scans, pc);
    std::cout << "Scans created : " << scans.size() << std::endl;


    PointCloud reconstruction_cloud;
    //this matrix holds all the points from the different scans
    Eigen::MatrixXd points_all_scans(3,1);
    writeReconstructedMeshToFile(scans, points_all_scans, reconstruction_cloud);

    std::cout << "Now we check for quality..." << std::endl;
    compareInputMeshWithReconstruction(pc, reconstruction_cloud);

//    std::cout << pc.m_numPoints << " points read\n";
//    std::cout << "Process 3d point cloud..." << std::endl;
//    // set the rotation and translation for the extrinsics of the camera
//    // change the Y rotation to move along the horizontal axis of the mesh
//    double rotationX_deg = 0.0, rotationY_deg = 270.0, rotationZ_deg = 0.0;
//    double translationX, translationY, translationZ;
//    // center the camera wrt to the point cloud
//    pc.getCenterXY(translationX, translationY);
//    translationZ = pc.computeRadiusFromCentroid();
//    // create a new camera object
//    Camera currentCamera(rotationX_deg,
//                         rotationY_deg,
//                         rotationZ_deg,
//                         translationX,
//                         translationY,
//                         translationZ);
//    pc.computeNearestProjectedPts(currentCamera);
//    pc.computeWorldCoordinates(currentCamera);
//    std::cout << "Writing 3D model to output folder..." << std::endl;
//    pc.writeCloud(MODEL_3_OUTPUT_FILENAME);
//    std::cout << "Magic!" << std::endl;



    return SUCCESS;
}