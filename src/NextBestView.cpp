//
// Created by Maria Ruxandra Robu on 26/05/2016.
//
#include <iostream>
#include "defs.h"
#include "utilities.h"
#include "NextBestView.h"
#include "Camera.h"
#include "PointCloud.h"



namespace nvp {
    void generateScansFromDegrees(std::vector<Camera> &scans, PointCloud &pc) {
        // Author: Karina Mady

        // set the rotation and translation for the extrinsics of the camera
        double rotationX_deg = 0.0, rotationY_deg = 0.0, rotationZ_deg = 0.0;
        double translationX, translationY, translationZ;

        //degree offset given by user
        double degrees_input = 0.0;

        std::cout << "Enter the shift in degrees between each scan: ";
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
            std::cout << "Scan: " << camNo+1 << " & Rotation y = " << rotationY_deg << std::endl;

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
            pcTransformed.write(outputFile);
            //std::cout << "File " << outputFile << " written" << std::endl;
        }
    }

    void getEstimatedReconstruction(std::vector<Camera> const &scans,
                                    Eigen::MatrixXd &out_mergedScans){
        // Author: Karina Mady
        Eigen::MatrixXd points_all_scans;

        std::cout << "\nStarting PCD reconstruction..." << std::endl;

        for (auto camNo = 0; camNo < scans.size(); ++camNo) {
            //for every scan create a point cloud and read the points in
            std::string inputFile = "../output/scan_" +
                                    std::to_string(camNo + 1) + ".ply";
            PointCloud currentPCD(inputFile);

            Camera currentCamera(scans[camNo]);
            currentPCD.computeWorldCoordinates(currentCamera);

            std::string outputFile = "../output/estimated_scan_" +
                                     std::to_string(camNo + 1) + ".ply";
            //std::cout << "Writing file " << outputFile << std::endl;
            currentPCD.write(outputFile);

            //this function takes each point cloud and concatenates them
            // into a single matrix with no duplicates
            mergePointCloudsNoDuplicates(currentPCD,points_all_scans);
        }
        out_mergedScans = points_all_scans;
    }


//compares the vertices between ic (original cloud) and rc (reconstructed cloud)
    void compareOriginalWithReconstruction(PointCloud &ic, PointCloud &rc) {
        // Author: Karina Mady

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

}// namespace nvp