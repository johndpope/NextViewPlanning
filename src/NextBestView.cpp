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
            std::cout << "Scan: " << camNo + 1 << " & Rotation y = " << rotationY_deg << std::endl;

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
                                    Eigen::MatrixXd &out_mergedScans) {
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
            mergePointCloudsNoDuplicates(currentPCD, points_all_scans);
        }
        out_mergedScans = points_all_scans;
    }

    void getEstimatedReconstructionFromKViews(PointCloud &pc,
                                              std::vector<Camera> const &kViews,
                                              Eigen::MatrixXd &out_mergedScans) {
        // this function doesn't write all the intermediary scans
        Eigen::MatrixXd points_all_scans;

        int k = int(kViews.size());
        for (auto camNo = 0; camNo < k; camNo++) {
            PointCloud pcTransformed(pc);

            // create a new camera object
            Camera currentCamera(kViews[camNo]);

            pcTransformed.computeNearestProjectedPts(currentCamera);

            pcTransformed.computeWorldCoordinates(currentCamera);

            mergePointCloudsNoDuplicates(pcTransformed, points_all_scans);

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

    double evaluateNBV(std::vector<Camera> &kplus1Views,
                       PointCloud &originalPCD) {
        // Author: Karina Mady
        Eigen::MatrixXd mergedScans;
        double scan_score = -1;

        PointCloud pcTransformed(originalPCD);

        // TODO: add the width of the zbuffer as a parameter -> higher
        getEstimatedReconstructionFromKViews(originalPCD,
                                             kplus1Views,
                                             mergedScans);

        //get the reconstructed point cloud
        PointCloud estimatedPCD(mergedScans);

        int numPointsOriginalPCD = int(originalPCD.m_numPoints);
        int numPointsEstimatedPCD = int(estimatedPCD.m_numPoints);

        std::cout << "The original cloud has " << numPointsOriginalPCD << " points" << std::endl;
        std::cout << "The reconstructed cloud has " << numPointsEstimatedPCD << " points" << std::endl;

        //compare the number of points in the reconstructed cloud
        // with the number of points in the original input cloud
        //The closer the value to 1 (equivalent to 100 %), the better.
        scan_score = double(numPointsEstimatedPCD) / double(numPointsOriginalPCD);

        printScoreToConsole(scan_score);

        return scan_score;
    }

    Camera getCameraFromDegrees(PointCloud &pc,
                              double &rotYDegrees) {
        // set the rotation and translation for the extrinsics of the camera
        double rotationX_deg = 0.0, rotationZ_deg = 0.0;
        double translationX, translationY, translationZ;
        pc.getCenterXY(translationX, translationY);
        translationZ = pc.computeRadiusFromCentroid();

        Camera currentCamera(rotationX_deg,
                            rotYDegrees,
                            rotationZ_deg,
                            translationX,
                            translationY,
                            translationZ);
        return currentCamera;
    }

    void getCameraVecFromDegrees(PointCloud &pc,
                                 Eigen::VectorXd &kYDegrees,
                                 std::vector<Camera> &out_kCamVect) {
        int k = int(kYDegrees.size());
        out_kCamVect.reserve(k);
        double rotationY_deg;

        for (int i = 0; i < k; i++) {
            rotationY_deg = kYDegrees[i];

            Camera currentCamera = getCameraFromDegrees(pc, rotationY_deg);
//            std::cout << "Adding camera #" << i << std::endl;
            out_kCamVect.push_back(currentCamera);
        }
    }

    int getNoStepsBetweenTwoViews(double prevYDegrees,
                                  double currentYDegrees,
                                  int stepsDegrees) {
        if (currentYDegrees < prevYDegrees) {
            // can happen when we consider the space between the last scan and the first one
            currentYDegrees += 360;
        }
        double differenceDegrees = currentYDegrees - prevYDegrees;
        return int(std::floor(differenceDegrees / stepsDegrees));
    }

    void getCandidateViewsBtTwoViews(Camera &prevView,
                                     Camera &currentView,
                                     int stepsDegrees,
                                     Eigen::VectorXd &out_candidateYDegrees,
                                     int &out_noCandidates) {
        double currentYDegrees = currentView.getRotationYDegrees();
        double prevYDegrees = prevView.getRotationYDegrees();

        int noAdditionalPos = getNoStepsBetweenTwoViews(prevYDegrees,
                                                        currentYDegrees,
                                                        stepsDegrees);
        for (int i = 0; i <= noAdditionalPos; i++) {
            double newPos = prevYDegrees + stepsDegrees * i;
            newPos = int(newPos) % 360;
            if (newPos != currentYDegrees && newPos != prevYDegrees) {
                out_candidateYDegrees[out_noCandidates] = newPos;
                out_noCandidates++;
            }
        }
    }

    void getCandidateViewsDegrees(std::vector<Camera> &kViews,
                                  Eigen::VectorXd &out_candidateYDegrees) {
        int noCandidates = 0;
        int k = int(kViews.size());
        int stepsDegrees = 50;

        out_candidateYDegrees = Eigen::VectorXd::Zero(int(std::floor(360 / stepsDegrees)));

        Camera prevView = kViews[0];

        for (int currK = 1; currK < k; currK++) {
            Camera currentView(kViews[currK]);

            getCandidateViewsBtTwoViews(prevView,
                                        currentView,
                                        stepsDegrees,
                                        out_candidateYDegrees,
                                        noCandidates);

            prevView = currentView;
        }
        // do the same thing for the space between the last camera and the first one;
        Camera currentView(kViews[0]);
        getCandidateViewsBtTwoViews(prevView,
                                    currentView,
                                    stepsDegrees,
                                    out_candidateYDegrees,
                                    noCandidates);
        out_candidateYDegrees.conservativeResize(noCandidates, 1);

        std::cout << "Found " << out_candidateYDegrees.size() << " candidate views\n";
        //std::cout << out_candidateYDegrees.transpose() << std::endl;

        // TODO: add new step with increased resolution + increase stepsDegrees in this case
    }

    int getNumNewPointsFromNewScan(PointCloud &pc,
                                   std::vector<Camera> &kViews,
                                   Camera &newView) {
        Eigen::MatrixXd estimatedPCD_kviews;

        // TODO: take this outside of the function, no need to compute it for each candidate
        // TODO: add the width of the zbuffer as a parameter -> lower
        getEstimatedReconstructionFromKViews(pc,
                                             kViews,
                                             estimatedPCD_kviews);

        Eigen::MatrixXd estimatedPCD_kplus1views;
        // build k+1 views vector
        std::vector<Camera> kplus1Views = getKplus1ViewVector(kViews,
                                                              newView);

        getEstimatedReconstructionFromKViews(pc,
                                             kplus1Views,
                                             estimatedPCD_kplus1views);

        // get the number of new points provided by the new scan by
        // subtracting the PCD obtained from the k+1 views and
        // the PCD obtained from the k views

        // note that the reconstructed PCD removes duplicates

        int numNewPoints = int(estimatedPCD_kplus1views.cols()) -
                           int(estimatedPCD_kviews.cols());

        return numNewPoints;
    }

    void evaluateEachCandidateView(PointCloud &pc,
                                   std::vector<Camera> &kViews,
                                   Eigen::VectorXd &candidateYDegrees,
                                   Eigen::VectorXd &out_viewScores) {
        int noCandidates = int(candidateYDegrees.size());
        out_viewScores = Eigen::VectorXd::Zero(noCandidates);
        // generate vector of camera position candidates
        std::vector<Camera> viewCandidates;
        getCameraVecFromDegrees(pc, candidateYDegrees, viewCandidates);

        for (int i = 0; i < noCandidates; i++) {
            std::cout << "Computing score for candidate view #" << i + 1 << "..." << std::endl;
            out_viewScores[i] = getNumNewPointsFromNewScan(pc,
                                                           kViews,
                                                           viewCandidates[i]);
        }
        std::cout << "Candidate views at: \n" << candidateYDegrees.transpose() << std::endl;
        std::cout << "Scores for the candidate views:\n" << out_viewScores.transpose() << std::endl;

    }


}// namespace nvp