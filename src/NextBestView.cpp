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
                                              Eigen::MatrixXd &out_mergedScans,
                                              int zbufferSideSize) {
        // this function doesn't write all the intermediary scans
        Eigen::MatrixXd points_all_scans;

        int k = int(kViews.size());
        for (auto camNo = 0; camNo < k; camNo++) {
            PointCloud pcTransformed(pc);

            // create a new camera object
            Camera currentCamera(kViews[camNo]);

            pcTransformed.computeNearestProjectedPts(currentCamera,
                                                     zbufferSideSize);

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
                       PointCloud &originalPCD,
                       int zbufferSideSize) {
        // Author: Karina Mady
        Eigen::MatrixXd mergedScans;
        double scan_score = -1;

        PointCloud pcTransformed(originalPCD);

        getEstimatedReconstructionFromKViews(originalPCD,
                                             kplus1Views,
                                             mergedScans,
                                             zbufferSideSize);

        //get the reconstructed point cloud
        PointCloud estimatedPCD(mergedScans);

        int numPointsOriginalPCD = int(originalPCD.m_numPoints);
        int numPointsEstimatedPCD = int(estimatedPCD.m_numPoints);

//        std::cout << "The original cloud has " << numPointsOriginalPCD << " points" << std::endl;
//        std::cout << "The reconstructed cloud has " << numPointsEstimatedPCD << " points" << std::endl;

        //compare the number of points in the reconstructed cloud
        // with the number of points in the original input cloud
        //The closer the value to 1 (equivalent to 100 %), the better.
        scan_score = double(numPointsEstimatedPCD) / double(numPointsOriginalPCD);

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

    void getInitialCandidateViewsDegrees(std::vector<Camera> &kViews,
                                         Eigen::VectorXd &out_candidateYDegrees) {
        // NOTE: this is the first step of the candidate view selection
        int noCandidates = 0;
        int k = int(kViews.size());
        int stepsDegrees = 70;

        out_candidateYDegrees = Eigen::VectorXd::Zero(int(std::floor(360 / stepsDegrees)));

        // ******************************************************
        // First step: very sparse sample of viewpoints around one axis
        // the scores for each view are computed and the best one is
        // selected for the next step
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

        std::cout << std::endl;
        std::cout << "Candidate search:\n\nGlobal sparse search - Found "
        << out_candidateYDegrees.size() << " candidate views\n";
        std::cout << out_candidateYDegrees.transpose() << std::endl;
    }

    Camera computeNBV(PointCloud &pc,
                      std::vector<Camera> &kViewVector,
                      bool FLAG_EVALwGT) {
        double NBV_degrees = 0;
        // ******************************************************
        // Get initial candidate views for the NBV - very sparse
        Eigen::VectorXd initialCandidViewYDegrees;
        getInitialCandidateViewsDegrees(kViewVector,
                                        initialCandidViewYDegrees);
        int noCandidates = int(initialCandidViewYDegrees.size());

        // ******************************************************
        // Compute scores for each new candidate view (no ground truth) =
        // the amount of new points brought by each candidate scan
        // NOTE: we use a lower resolution of each scan to make the computation faster
        // TODO: add score based on point quality as well
        Eigen::VectorXd scoresCandidateViews;
        evaluateEachCandidateView(pc,
                                  kViewVector,
                                  initialCandidViewYDegrees,
                                  scoresCandidateViews);

        // get maximum score and use that candidate view for
        // the search in local neighbourhood
        int idxMaxInit = 0;
        double initMaxScore = getMaxFromEigVector(scoresCandidateViews, idxMaxInit);
        std::cout << "Global sparse search - Found maximum score of " << scoresCandidateViews[idxMaxInit] <<
        " for " << initialCandidViewYDegrees[idxMaxInit] << " degrees\n";

        if (FLAG_EVALwGT) {
            // ******************************************************
            // ********************** OPTIONAL **********************
            // Check with the ground truth if we actually chose the best candidate view
            int zbufferSideSize = 150;
            Eigen::VectorXd scoresGTVec(noCandidates);
            for (int i = 0; i < noCandidates; i++) {
                std::cout << "Eval with GT - Compute score for candidate view #" << i + 1 << ": ";
                Camera kplus1View_temp = getCameraFromDegrees(pc,
                                                              initialCandidViewYDegrees[i]);

                // create a temporary vector of k+1 views for each candidate position
                std::vector<Camera> kplus1ViewVector_temp = getKplus1ViewVector(kViewVector,
                                                                                kplus1View_temp);
                scoresGTVec[i] = evaluateNBV(kplus1ViewVector_temp,
                                             pc,
                                             zbufferSideSize);
                std::cout << scoresGTVec[i] << std::endl;
            }
            int idxMaxScoreGT;
            double maxScoreGT = getMaxFromEigVector(scoresGTVec, idxMaxScoreGT);
            std::cout << "Eval with GT - Found maximum score of " << scoresGTVec[idxMaxScoreGT] <<
            " for " << initialCandidViewYDegrees[idxMaxScoreGT] << " degrees\n";
        }

        NBV_degrees = initialCandidViewYDegrees[idxMaxInit];
        // if you're happy with this initial search, uncomment the next line
        // return getCameraFromDegrees(pc,NBV_degrees);

        std::cout << std::endl;
        // ******************************************************
        // Use the candidate view with the best score to search for
        // a better solution in its neighbourhood
        double localSearchDegrees = 20.0;
        int numLocalCandid = int(localSearchDegrees * 2 / 10) + 1;
        Eigen::VectorXd finalCandidViewYDegrees =
                Eigen::VectorXd::LinSpaced(numLocalCandid,
                                           -localSearchDegrees,
                                           +localSearchDegrees);
        // add degrees in the candidate view vector relative to the new max
        finalCandidViewYDegrees = NBV_degrees * Eigen::VectorXd::Ones(numLocalCandid)
                                  + finalCandidViewYDegrees;
        // all of the numbers should be between 0 and 360 degrees!
        for (int i = 0; i < numLocalCandid; i++) {
            // -10 ends up being 350
            // 370 ends up being 20
            if(finalCandidViewYDegrees[i] < 0)
                finalCandidViewYDegrees[i] += 360 ;
            else if(finalCandidViewYDegrees[i] < 0)
                finalCandidViewYDegrees[i] -= 360;
        }
        std::cout << "Local search - Found new candidate views\n"<<
                finalCandidViewYDegrees.transpose() << std::endl;

        Eigen::VectorXd finalScoresCandidViews;
        evaluateEachCandidateView(pc,
                                  kViewVector,
                                  finalCandidViewYDegrees,
                                  finalScoresCandidViews);
        int idxMaxFin = 0;
        double finMaxScore = getMaxFromEigVector(finalScoresCandidViews, idxMaxFin);
        std::cout << "Local search - Found maximum score of " << finMaxScore <<
        " for " << finalCandidViewYDegrees[idxMaxFin] << " degrees\n";
        // ******************************************************
        // if the final maximum score is better than the one from the initial step
        // use the new view as the NBV
        if (finMaxScore > initMaxScore)
        {
            NBV_degrees = finalCandidViewYDegrees[idxMaxFin];

        }
        else // initMaxScore >= finMaxScore
        {
            finMaxScore = initMaxScore;
            NBV_degrees = initialCandidViewYDegrees[idxMaxInit];

        }
        std::cout << "Final NEXT BEST VIEW = " << NBV_degrees
        << " with a score of " << finMaxScore << std::endl << std::endl;
        return getCameraFromDegrees(pc, NBV_degrees);
    }

    int getNumNewPointsFromNewScan(PointCloud &pc,
                                   std::vector<Camera> &kViews,
                                   Camera &newView,
                                   int numPoints_kViews,
                                   int zbufferSideSize) {

        Eigen::MatrixXd estimatedPCD_kplus1views;
        // build k+1 views vector
        std::vector<Camera> kplus1Views = getKplus1ViewVector(kViews,
                                                              newView);

        getEstimatedReconstructionFromKViews(pc,
                                             kplus1Views,
                                             estimatedPCD_kplus1views,
                                             zbufferSideSize);

        // get the number of new points provided by the new scan by
        // subtracting the PCD obtained from the k+1 views and
        // the PCD obtained from the k views

        // ! the reconstructed PCD has no duplicates

        int numNewPoints = int(estimatedPCD_kplus1views.cols()) - numPoints_kViews;

        return numNewPoints;
    }

    void evaluateEachCandidateView(PointCloud &pc,
                                   std::vector<Camera> &kViews,
                                   Eigen::VectorXd &candidateYDegrees,
                                   Eigen::VectorXd &out_viewScores) {
        int zbufferSideSize = 50;
        int noCandidates = int(candidateYDegrees.size());
        out_viewScores = Eigen::VectorXd::Zero(noCandidates);
        // generate vector of camera position candidates
        std::vector<Camera> viewCandidates;
        getCameraVecFromDegrees(pc, candidateYDegrees, viewCandidates);

        // get reconstruction from k views
        Eigen::MatrixXd estimatedPCD_kviews;

        getEstimatedReconstructionFromKViews(pc,
                                             kViews,
                                             estimatedPCD_kviews,
                                             zbufferSideSize);

        int numPoints_kViews = int(estimatedPCD_kviews.cols());

        for (int i = 0; i < noCandidates; i++) {
            std::cout << "Computing score for candidate view #" << i + 1 << "..." << std::endl;
            out_viewScores[i] = getNumNewPointsFromNewScan(pc,
                                                           kViews,
                                                           viewCandidates[i],
                                                           numPoints_kViews,
                                                           zbufferSideSize);
        }
        std::cout << "Candidate views at: \n" << candidateYDegrees.transpose() << std::endl;
        std::cout << "Scores for the candidate views:\n" << out_viewScores.transpose() << std::endl;

    }


}// namespace nvp