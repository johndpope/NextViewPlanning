//
// Created by Maria Ruxandra Robu on 26/05/2016.
//

#ifndef NEXTVIEWPLANNING_NEXTBESTVIEW_H
#define NEXTVIEWPLANNING_NEXTBESTVIEW_H

#include <Eigen/Dense>
#include <vector>

namespace nvp {
    //*************** Core Framework *******************

    // This function generates scans and back-projections using an offset
    // given by the user to rotate around the y axis
    // This is used to test the core framework
    // Author: Karina Mady
    void generateScansFromDegrees(std::vector<Camera> &scans,
                                  PointCloud &pc);

    // This function estimates the final reconstruction by merging all
    // the scans in a point cloud without duplicates
    // All the scans are written to .ply files
    // Author: Karina Mady
    void getEstimatedReconstruction(std::vector<Camera> const &scans,
                                    Eigen::MatrixXd &out_mergedScans);

    // This function compares the number of vertices from the reconstructed
    // and the original model
    // Author: Karina Mady
    void compareOriginalWithReconstruction(PointCloud &ic,
                                           PointCloud &rc);

    // This function evaluates the next best view (NBV) through comparison
    // with the ground truth
    // Author: Karina Mady
    double evaluateNBV(std::vector<Camera> &kplus1Views,
                       PointCloud &originalPCD,
                       int zbufferSideSize = 200);

    //*************** Individual Section *******************

    // This function estimates the final reconstruction by merging all
    // the scans in a point cloud without duplicates
    // No writing to .ply files
    void getEstimatedReconstructionFromKViews(PointCloud &pc,
                                              std::vector<Camera> const &kViews,
                                              Eigen::MatrixXd &out_mergedScans,
                                              int zbufferSideSize = 100);

    // This function outputs a Camera object created from a given rotation in
    // degrees around the y axis
    Camera getCameraFromDegrees(PointCloud &pc,
                                double &rotYDegrees);

    // This function generates the vector of Cameras for the k views
    void getCameraVecFromDegrees(PointCloud &pc,
                                 Eigen::VectorXd &kYDegrees,
                                 std::vector<Camera> &out_kCamVect);

    // This function computes the number of candidate views needed between
    // 2 cameras in the global search step
    int getNoStepsBetweenTwoViews(double prevYDegrees,
                                  double currentYDegrees,
                                  int stepsDegrees);

    // This function computes the candidate views (in degrees) between 2 cameras
    // in the global search step
    void getCandidateViewsBtTwoViews(Camera &prevView,
                                     Camera &currentView,
                                     int stepsDegrees,
                                     Eigen::VectorXd &out_candidateYDegrees,
                                     int &out_noCandidates);

    // This function computes the candidate views for the global search step
    void getInitialCandidateViewsDegrees(std::vector<Camera> &kViews,
                                         Eigen::VectorXd &out_candidateYDegrees);


    // Monster function that outputs the best NBV and calls all the other
    // helper functions
    Camera computeNBV(PointCloud &pc,
                      std::vector<Camera> &kViewVector,
                      bool FLAG_EVALwGT = false);

    // Score_N = evaluation of a candidate view based on the number of
    // new points it brings
    int getNumNewPointsFromNewScan(PointCloud &pc,
                                   std::vector<Camera> &kViews,
                                   Camera &newView,
                                   int numPoints_kViews,
                                   int zbufferSideSize = 100);
    // Score_Q = evaluation of a candidate view based on the quality of
    // the points is brings
    double getQualityScoreFromNewScan(PointCloud &pc,
                                      Camera &newView,
                                      int zbufferSideSize = 100);

    // This function evaluates each candidate view based on a linear
    // combination between the 2 scores: Score_N and Score_Q
    void evaluateEachCandidateView(PointCloud &pc,
                                   std::vector<Camera> &kViews,
                                   Eigen::VectorXd &candidateYDegrees,
                                   Eigen::VectorXd &out_viewScores);


}// namespace nvp
#endif //NEXTVIEWPLANNING_NEXTBESTVIEW_H
